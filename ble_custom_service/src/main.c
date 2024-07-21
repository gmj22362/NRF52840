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

#include "ble.h"
#include "uart.h"
#include "custom_srv.h"

LOG_MODULE_REGISTER(main);

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	printk("button_changed %x\r\n", button_state);

	if (buttons & DK_BTN1_MSK) {
		printk("button1 press\r\n");
	}
	if (buttons & DK_BTN2_MSK) {
		printk("button2 press\r\n");
	}
}


static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	uart_init();

	ble_init();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	static uint8_t tmp[5] = "1234"; 
	k_sem_take(&ble_init_ok, K_FOREVER);
	
	for (;;) {
		if(current_conn){
			indicate_send(current_conn, tmp, 5);
			bt_custom_send(current_conn, tmp, 3);
		}
		tmp[0] ++;
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

K_THREAD_DEFINE(ble_write_thread_id, BT_THREAD_STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, BT_THREAD_PRIORITY, 0, 0);


