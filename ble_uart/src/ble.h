#ifndef _BLE_H_
#define _BLE_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#define CON_STATUS_LED DK_LED2

#define BT_NUS_STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define BT_NUS_PRIORITY 7

#define BT_DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define BT_DEVICE_NAME_LEN	(sizeof(BT_DEVICE_NAME) - 1)

extern struct k_sem ble_init_ok;

int ble_init(void);

#endif // !_BLE_H_
