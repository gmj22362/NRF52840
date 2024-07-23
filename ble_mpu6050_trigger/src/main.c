/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <dk_buttons_and_leds.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>

#include "ble.h"
#include "uart.h"
#include "custom_srv.h"


LOG_MODULE_REGISTER(main);


#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

// Verify that there is exactly one MPU6050 device
#define MPU6050_NODE DT_NODELABEL(mpu6050)

#if !DT_NODE_HAS_STATUS(MPU6050_NODE, okay)
#error "MPU6050 device node is not okay"
#endif


static K_SEM_DEFINE(sem, 0, 1);

static void mpu_trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	switch (trigger->type) {
	case SENSOR_TRIG_DATA_READY:
		printk("Data ready trigger\r\n");
		break;
	default:
		printk("Unknown trigger event %d\r\n", trigger->type);
		break;
	}
	k_sem_give(&sem);
}

static void configure_gpio(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

/* This sets up  the sensor to signal valid data when a threshold
 * value is reached.
 */
static void process(const struct device *dev)
{
	int ret;
	struct sensor_value temp_val[3];
	struct sensor_trigger sensor_trig_conf = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_RED,
	};

	if (sensor_trigger_set(dev, &sensor_trig_conf,
			mpu_trigger_handler)) {
		printk("Could not set trigger\n");
		return;
	}
	

	while (1) {
		k_sem_take(&sem, K_FOREVER);

		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
		/* The sensor does only support fetching SENSOR_CHAN_ALL */
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return;
		}

		ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, temp_val);
		if (ret) {
			printk("sensor_channel_get failed ret %d\n", ret);
			return;
		}
		printk("mpu6050 XYZ: %x,%x,%x \n", temp_val[0].val1, temp_val[1].val1, temp_val[2].val1);


		k_sleep(K_MSEC(2000));
	}
}

int main(void)
{
	int blink_status = 0;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT)) {
		/* Give RTT log time to be flushed before executing tests */
		k_sleep(K_MSEC(500));
	}

	configure_gpio();

	uart_init();

	ble_init();

	const struct device *dev = DEVICE_DT_GET(MPU6050_NODE);

	//设置中断速率
	

	process(dev);
}

void ble_write_thread(void)
{
	static uint8_t tmp[5] = "1234"; 
	k_sem_take(&ble_init_ok, K_FOREVER);
	
	for (;;) {
		if(current_conn){
			indicate_send(current_conn, tmp, 5);
			bt_custom_send(current_conn, tmp, 2);
		}
		tmp[0] ++;
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

K_THREAD_DEFINE(ble_write_thread_id, BT_THREAD_STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, BT_THREAD_PRIORITY, 0, 0);


