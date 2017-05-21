/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
  * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// interval 
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <arch/byteorder.h>
#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/device.h>
#include <nuttx/device_raw.h>
#include <nuttx/power/pm.h>
#include <nuttx/time.h>
#include <time.h>
#include <nuttx/wqueue.h>
#include <arch/board/mods.h>

#include "arch/board/th02.h"

#include "hdk.h"
#include "aes.h"
#define BLINKY_ACTIVITY    10
#define BLINKY_TIM          6
#define BLINKY_TIM_FREQ  1000
#define BLINKY_PERIOD    1000

#define LED_ON              0
#define LED_OFF             1

#ifdef CONFIG_STM32_ADC1
/* The number of ADC channels in the conversion list */
#define ADC1_NCHANNELS 1

/* Identifying number of each ADC channel
 * Temperature output is available on PC3, which corresponds
 * to ADC1 channel 4 (GPIO_ADC1_IN4).
 */
static const uint8_t  g_chanlist[ADC1_NCHANNELS] = {4};

/* Configurations of pins used by each ADC channel */
static const uint32_t g_pinlist[ADC1_NCHANNELS]  = {GPIO_ADC1_IN4};
#endif

static struct work_s data_report_work; /* work queue for data reporting */


#define TEMP_RAW_NAME     "TEMPERATURE"
#define TEMP_RAW_DRIVER_VERSION   0x01
#define TEMP_RAW_MAX_LATENCY      2000    /* milliSeconds */

/* Command IDs */
#define TEMP_RAW_COMMAND_INVALID  0x00
#define TEMP_RAW_COMMAND_INFO     0x01
#define TEMP_RAW_COMMAND_ON       0x02
#define TEMP_RAW_COMMAND_OFF      0x03
#define TEMP_RAW_COMMAND_DATA     0x04
#define TEMP_RAW_COMMAND_CHALLENGE      0x05
#define TEMP_RAW_COMMAND_CHLGE_RESP     0x06

#define TEMP_RAW_COMMAND_RESP_MASK    0x80

/* AES */
#define TEMP_RAW_AES_KEY            "moto-temp-apk"
#define TEMP_RAW_AES_BLOCK_SIZE     16
#define TEMP_RAW_AES_CLIENT_ADDS    777

/* AES key, encrypted and decrypted data */
struct temp_raw_aes {
    char    key[TEMP_RAW_AES_BLOCK_SIZE];
    uint8_t input[TEMP_RAW_AES_BLOCK_SIZE];
    uint8_t encrypted_data[TEMP_RAW_AES_BLOCK_SIZE];
    uint8_t decrypted_data[TEMP_RAW_AES_BLOCK_SIZE];
};

struct temp_raw_info {
    struct device *gDevice;         /* device handle for this driver */
    uint16_t interval;              /* reporting interval */
    raw_send_callback gCallback;    /* callback to send back messages */
    uint8_t client_verified;        /* flag to indicate client was verified */
    struct temp_raw_aes tr_aes;     /* device aes structure */
};

static int adc_devinit(void)
{
#ifdef CONFIG_STM32_ADC1
    static bool initialized = false;
    struct adc_dev_s *adc;
    int ret;
    int i;


    dbg("Entering ADC init function");
    /* Check if we have already initialized */
    if (!initialized)
    {
        /* Configure the pins as analog inputs for the selected channels */
        for (i = 0; i < ARRAY_SIZE(g_pinlist); i++)
        {
            stm32_configgpio(g_pinlist[i]);
        }

        /* Call stm32_adcinitialize() to get an instance of the ADC interface */
        adc = stm32_adcinitialize(1, g_chanlist, ARRAY_SIZE(g_chanlist));
        if (adc == NULL)
        {
            dbg("ERROR: Failed to get ADC interface\n");
            return -ENODEV;
        }

        /* Register the ADC driver */
        ret = adc_register(TEMP_RAW_ADC_DEVPATH, adc);
        if (ret < 0)
        {
            dbg("adc_register failed: %d\n", ret);
            return ret;
        }

        /* Now we are initialized */
        dbg("adc initialized: %s\n", TEMP_RAW_ADC_DEVPATH);
        initialized = true;
    }

    return OK;
#else
    return -ENOSYS;
#endif
}

struct blinky_info {
    struct device *gDevice;
    raw_send_callback gCallback;
};
static int my_send(struct device *dev, uint32_t id, float value)
{
    dbg("MY SEND FUNCTION\n\n");

    struct blinky_info *info = NULL;
    uint8_t *resp_msg;
    union {
        float a;
        uint8_t bytes[4];
    } thing;
    thing.a = value;

    dbg("Sending response for %d \n", id);

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);
        
    // allocate memory for response message 
    resp_msg = zalloc(5);
    if (!resp_msg) {
        return -ENOMEM;
    }
    *resp_msg = id;
    memcpy((void *)(resp_msg + 1), thing.bytes, 4);

    dbg("Response: %d %d %d %d %d \n", resp_msg[0], resp_msg[1], resp_msg[2],
        resp_msg[3], resp_msg[4]);

    if (info->gCallback) {
        info->gCallback(info->gDevice, 5, resp_msg);
    }

    free(resp_msg);
    return OK;
}

static void adc_worker(void *arg)
{
    int ret;
    int adc_fd;
    struct adc_msg_s sample;
    struct temp_raw_info *info = NULL;


    //info = device_get_private(arg);
    info = arg;


    dbg("ADC WORKER CALLED\n");

    adc_fd = open(TEMP_RAW_ADC_DEVPATH, O_RDONLY);
    if (adc_fd < 0)
    {
        dbg("open %s failed: %d\n", TEMP_RAW_ADC_DEVPATH, errno);
        goto errout;
    }

    ret = ioctl(adc_fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
        dbg("ANIOC_TRIGGER ioctl failed: %d\n", errno);
    }

    read(adc_fd, &sample, sizeof(sample));
    close(adc_fd);

    dbg("Read from ADC:  %d\n", sample.am_data);
    
    struct timespec delay;
    clock_gettime(CLOCK_REALTIME, &delay);
    uint32_t firstTime = timespec_to_usec(&delay);

int count = 0;
    while(sample.am_data < 2058) {
      count++;
      read(adc_fd, &sample, sizeof(sample));
      if (count>10000) break;
     }

clock_gettime(CLOCK_REALTIME, &delay);
uint32_t secondTime = timespec_to_usec(&delay);
float cap;
float resistorValue = 100000.0;
cap = (float) ((secondTime - firstTime) / resistorValue);

    dbg("Creating ADC msg...");
    dbg("Sending ADC data via RAW...");
    my_send(info->gDevice, 0, (float) cap);
    dbg("Finished sending adc data.");



    /* cancel any work and reset ourselves */
    if (!work_available(&data_report_work))
        work_cancel(LPWORK, &data_report_work);

    /* schedule work */
    work_queue(LPWORK, &data_report_work,
                adc_worker, info, MSEC2TICK(40000));
errout:
    return;
}



static int my_recv(struct device *dev, uint32_t len, uint8_t data[])
{
   
    struct temp_raw_info *info = NULL;
    info = device_get_private(dev);
    int res = OK;
    float temperature_result;
    float humidity_result;;

    dbg("> MY_RECV FUNC...\n\n");

    //getTemperature();
    //getHumidity();
    

        if (data[0] == 0 || data[0] == '0'){
            dbg("RECEIVED 0\n\n");

            if (!work_available(&data_report_work))
                work_cancel(LPWORK, &data_report_work);
            //  schedule work 
            work_queue(LPWORK, &data_report_work,
                        adc_worker, info, 0);
       //     my_send(dev, 0, 101.0);
        }
        else if (data[0] == 1 || data[0] == '1'){
            dbg("RECEIVED 1\n\n");
            
            temperature_result = getTemperature();
            dbg("Creating temperature msg...");
            dbg("Sending Temperature data via RAW...");
            my_send(dev, 1, (float)temperature_result);
            dbg("Finished sending temperature data.");
        }
        else if (data[0] == 2 || data[0] == '2'){
            dbg("RECEIVED 2\n\n");
            humidity_result = getHumidity();
            dbg("Creating Humidity msg...");
            dbg("Sending Humidty data via RAW...");
            my_send(dev, 2, (float)humidity_result);
            dbg("Finished sending humidity data.");
        }
        else if (data[0] == 3 || data[0] == '3'){
            dbg("RECEIVED 3\n\n");
            if (!work_available(&data_report_work))
                work_cancel(LPWORK, &data_report_work);
        } 

        return res;



    return -EINVAL;
}

static int my_register_callback(struct device *dev,
                                    raw_send_callback callback)
{
    struct blinky_info *info = NULL;
    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    info->gCallback = callback;
    return 0;
}

static int my_unregister_callback(struct device *dev)
{
    struct blinky_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -ENODEV;
    }
    info = device_get_private(dev);

    info->gCallback = NULL;	
    return 0;
}

static int my_setup(struct device *dev)
{
    dbg("Running on Setup...24\n");
    adc_devinit();
    enable_TH02(3, 64);

    gpio_direction_out(GPIO_MODS_LED_DRV_3, LED_OFF);
    //gpio_direction_out(GPIO_MYGPIO2, 0);
    //gpio_direction_out(GPIO_MYGPIO3, 1);
    //gpio_direction_in(GPIO_MODS_BUTTON);

    struct blinky_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->gDevice = dev;
    device_set_private(dev, info);

    dbg("> SETUP FINISHED...12\n\n");

    return 0;
}

/**
 * remove function called when device is unregistered.
 */
static void my_remove(struct device *dev)
{
    struct blinky_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    free(info);
    device_set_private(dev, NULL);
}


static struct device_raw_type_ops testadc_type_ops = {
    .recv = my_recv,
    .register_callback = my_register_callback,
    .unregister_callback = my_unregister_callback,
};

static struct device_driver_ops testadc_driver_ops = {
    .probe = my_setup,
    .remove = my_remove, 
    .type_ops = &testadc_type_ops,
};

struct device_driver mods_raw_blinky_driver = {
    .type = DEVICE_TYPE_RAW_HW,
    .name = "mods_testadc",
    .desc = "MOD Testadc by Luan",
    .ops = &testadc_driver_ops,
};
