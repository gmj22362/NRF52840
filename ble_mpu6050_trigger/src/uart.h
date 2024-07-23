#ifndef _UART_H_
#define _UART_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <soc.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
#include "uart_async_adapter.h"

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

extern const struct device *uart;
extern struct k_fifo fifo_uart_tx_data;
extern struct k_fifo fifo_uart_rx_data;


int uart_tx_by_fifo(const struct device *uart, const uint8_t *const data, uint16_t len);
int uart_init(void);

#endif // !_UART_H_
