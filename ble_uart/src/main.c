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

LOG_MODULE_REGISTER(main);

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK


void error(void)
{
	while (true) {
		/* Spin for ever */
		printk("error while!\r\n");
		k_sleep(K_MSEC(1000));
	}
}

extern struct bt_conn *auth_conn;
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	printk("button_changed %x\r\n", button_state);
	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
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

	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);


		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}else{
			LOG_INF("Send %d bytes data to BLE", buf->len);
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, BT_NUS_STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, BT_NUS_PRIORITY, 0, 0);
