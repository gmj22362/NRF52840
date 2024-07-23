#ifndef _CUSTOM_SRV_H_
#define _CUSTOM_SRV_H_

#include "ble.h"

int indicate_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);
int bt_custom_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);


#endif // !_CUSTOM_SRV_H_

