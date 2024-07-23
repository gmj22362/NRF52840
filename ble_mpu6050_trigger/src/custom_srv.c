#include "custom_srv.h"

LOG_MODULE_REGISTER(ble_custom_srv);

#define PRIMARY_SERVICE_UUID BT_UUID_DECLARE_16(0x4321) 
#define INDICATE_CHAR_UUID BT_UUID_DECLARE_16(0x4322)
#define WRITE_CHAR_UUID BT_UUID_DECLARE_16(0x4323)
#define NOTIFY_CHAR_UUID BT_UUID_DECLARE_16(0x4324)

bool indicate_flag = false;
bool indicate_enabled = false;

bool notify_enabled = false;
bool notify_flag = false;

static ssize_t indicate_char_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                      uint16_t len, uint16_t offset)
{
    // 获取存储在属性中的用户数据
    const char *value = attr->user_data;

    // 打印读取操作的相关信息
    printk("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

    // 使用 bt_gatt_attr_read 函数读取属性值并返回读取的数据长度
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static void indicate_char_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    indicate_enabled = (value == BT_GATT_CCC_INDICATE);
}

static void notify_char_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t write_char_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                   uint16_t len, uint16_t offset, uint8_t flags)
{
    // 打印写入操作的相关信息
    printk("Attribute(write_char) write, handle: %u, conn: %p, len:%d\n", attr->handle, (void *)conn, len);

    // 检查数据偏移量是否正确
    if (offset != 0) {
        printk("Write led: Incorrect data offset\n");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    // 将写入的数据转换为字符串格式
    char *value = (uint8_t *)buf;
    printk("Write data: %.*s\n", len, value);

    // 返回写入的数据长度
    return len;
}

//自定义蓝牙 GATT（Generic Attribute Profile）服务
BT_GATT_SERVICE_DEFINE(
	// 这个宏定义了一个 GATT 服务，并将其命名为 custom_service。
	// custom_service 是这个服务的标识符，将用于引用和操作该服务。
    custom_service, 

	// 这个宏定义了一个主服务（Primary Service）。
	// PRIMARY_SERVICE_UUID 是服务的 UUID，用于唯一标识该服务。
    BT_GATT_PRIMARY_SERVICE(PRIMARY_SERVICE_UUID),

    //可读通知特征
    BT_GATT_CHARACTERISTIC(
        NOTIFY_CHAR_UUID, 
        BT_GATT_CHRC_NOTIFY, 
        BT_GATT_PERM_READ, 
        NULL,   //读操作的回调函数
        NULL,   //写操作的回调函数
        NULL    //初始值或状态指针
    ),
	BT_GATT_CCC(
        notify_char_ccc_cfg_changed, 
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
    ),

    //可读指示性特征
    BT_GATT_CHARACTERISTIC(
        INDICATE_CHAR_UUID, 
        BT_GATT_CHRC_INDICATE, 
        BT_GATT_PERM_READ, 
        indicate_char_callback, 
        NULL, 
        &indicate_flag
    ),
    BT_GATT_CCC(
        indicate_char_ccc_cfg_changed, 
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE
    ),

    //写特征
    BT_GATT_CHARACTERISTIC(
        WRITE_CHAR_UUID, 
        BT_GATT_CHRC_WRITE, 
        BT_GATT_PERM_WRITE, 
        NULL, 
        write_char_callback, 
        NULL
    ),
);

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
    printk("Indication %s\n", err != 0U ? "fail" : "success");
}


int indicate_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    static struct bt_gatt_indicate_params indicate_param;
	if(!indicate_enabled)
	{
		return EACCES;
	}

	indicate_param.attr = &custom_service.attrs[4];
	indicate_param.func = indicate_cb;
	indicate_param.data = data;
	indicate_param.len = len;
	indicate_param.destroy = NULL;

	return bt_gatt_indicate(conn, &indicate_param);
}

static void on_sent(struct bt_conn *conn, void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_DBG("Data send, conn %p", (void *)conn);

}

int bt_custom_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	struct bt_gatt_notify_params params = {0};
	const struct bt_gatt_attr *attr = &custom_service.attrs[2];

	params.attr = attr;
	params.data = data;
	params.len = len;
	params.func = on_sent;

	if (!conn) {
		LOG_DBG("Notification send to all connected peers");
		return bt_gatt_notify_cb(NULL, &params);
	} else if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		return bt_gatt_notify_cb(conn, &params);
	} else {
		return -EINVAL;
	}
}

