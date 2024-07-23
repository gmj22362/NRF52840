#include "mpu6050.h"

LOG_MODULE_REGISTER(mpu6050);

int data_cnt = 0;

/* measured in degrees/sec x10 to avoid floating point */
static const uint16_t mpu6050_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

static const uint8_t mpu6050_init_cmd[11][2] = {
    {PWR_MGMT_1, 0x80}, // PWR_MGMT_1, DEVICE_RESET  
    // need wait 
    {PWR_MGMT_1, 0x00}, // cleat SLEEP
    {GYRO_CONFIG, 0x18}, // Gyroscope Full Scale Range = ± 2000 °/s
    {ACCEL_CONFIG, 0x00}, // Accelerometer Full Scale Range = ± 2g 
    {INT_ENABLE, 0x01}, // Interrupt Enable.  DATA_RDY_EN
    {USER_CTRL, 0x00}, // User Control.auxiliary I2C are logically driven by the primary I2C bus
    {FIFO_EN, 0x00}, // FIFO Enable.disenable  
    {SMPLRT_DIV, 9}, // Sample Rate Divider.Sample Rate = 1KHz / (1 + 9)  
    {CONFIG, 0x23}, // EXT_SYNC_SET = GYRO_ZOUT_L[0]; Bandwidth = 3
    {PWR_MGMT_1, 0x01}, // Power Management 1.PLL with X axis gyroscope reference
    {PWR_MGMT_2, 0x00}  // Power Management 2
};

static struct gpio_dt_spec mpu6050_int = 
    GPIO_DT_SPEC_GET(DT_NODELABEL(mpu6050), int_gpios);

mpu6050_info_t mpu6050_dev = {
	// .i2c_drv = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .dev_addr = MPU_SENSOR_ADDR,
	.soft_i2c_drv ={
		.name = "mpu6050_i2c",
		.clk_period = 25,
		.sda_pin = I2C_SDA_PIN,
		.scl_pin = I2C_SCL_PIN,
	},
#ifdef  MPU6050_INT_ENABLE  
    .int_gpio = &mpu6050_int,
#endif
};

#ifdef  MPU6050_INT_ENABLE  
// 中断处理程序
static void mpu6050_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	mpu6050_info_t *info = CONTAINER_OF(cb, mpu6050_info_t, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(info->int_gpio, GPIO_INT_DISABLE);

#if defined(MPU6050_TRIGGER_OWN_THREAD)
	k_sem_give(&info->gpio_sem);
#elif defined(MPU6050_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&info->work);
#endif
}
#endif

/*
 * @description: mpu6050读取寄存器数据 
 * @param:          reg_addr    寄存器地址  
 *                  data        数据缓存
 * @return:         无
 */
void mpu_read_reg(mpu6050_info_t *dev, uint8_t reg_addr, uint8_t *data)
{
	i2c_read_reg(&dev->soft_i2c_drv, dev->dev_addr, reg_addr, data, 1);
}

/*
 * @description: mpu6050写寄存器数据 
 * @param:          reg_addr    寄存器地址  
 *                  data        数据缓存
 * @return:         无
 */
void mpu_write_reg(mpu6050_info_t *dev, uint8_t reg_addr, uint8_t data)
{
    i2c_write_reg(&dev->soft_i2c_drv, dev->dev_addr, reg_addr, &data, 1);
}

/*
 * @description: mpu6050读取寄存器数据 
 * @param:          reg_addr    寄存器地址  
 *                  data        数据缓存
 *                  len         字节数
 * @return:         无
 */
void mpu_read_n_reg(mpu6050_info_t *dev, uint8_t reg_addr, uint8_t* data, uint32_t len)
{
	i2c_read_reg(&dev->soft_i2c_drv, dev->dev_addr, reg_addr, data, len);
}

/* see "Accelerometer Measurements" section from register map description */
/*
 * @description: 转换加速度数据 
 * @param:       val               传感器值结构体指针
 *               raw_val           原始加速度值
 *               sensitivity_shift 灵敏度移位值
 * @return:      无
 */
static void mpu6050_convert_accel(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
/*
 * @description: 转换陀螺仪数据 
 * @param:       val                传感器值结构体指针
 *               raw_val            原始陀螺仪值
 *               sensitivity_x10    灵敏度x10
 * @return:      无
 */
static void mpu6050_convert_gyro(struct sensor_value *val, int16_t raw_val,
				 uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
/*
 * @description: 转换温度数据 
 * @param:       val        传感器值结构体指针
 *               raw_val    原始温度值
 * @return:      无
 */
static inline void mpu6050_convert_temp(struct sensor_value *val,
					int16_t raw_val)
{
	val->val1 = raw_val / 340 + 36;
	val->val2 = ((int64_t)(raw_val % 340) * 1000000) / 340 + 530000;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

/*
 * @description: 转换MPU6050传感器通道数据 
 * @param:       info       MPU6050设备结构体指针
 *               chan       传感器通道
 *               val        传感器值结构体指针
 * @return:      0          成功
 *              -ENOTSUP    不支持的通道
 */
int mpu6050_channel_get(mpu6050_info_t *info,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	mpu6050_data_t *drv_data = &info->sensor_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		mpu6050_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		mpu6050_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		mpu6050_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		mpu6050_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		mpu6050_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		mpu6050_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		mpu6050_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		mpu6050_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		mpu6050_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		mpu6050_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		mpu6050_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		mpu6050_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		mpu6050_convert_temp(val, drv_data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * @description: struct sensor_value 数据打印
 * @param:          value  struct sensor_value 数据
 * @return:         无
 */
void print_sensor_value(struct sensor_value *value) {
    // Check if the fractional part is negative
    if (value->val2 < 0) {
        // Print negative fractional part correctly
        printk("%d.%06d\n", value->val1, -value->val2);
    } else {
        // Print normally
        printk("%d.%06d\n", value->val1, value->val2);
    }
}

#ifdef MPU6050_TRIGGER_OWN_THREAD
/*
 * @description: mpu6050中断读取数据线程 
 * @param:          无  
 * @return:         无
 */
static void mpu6050_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	mpu6050_info_t *info = p1;

	while (1) {
        //等待接收
		k_sem_take(&info->gpio_sem, K_FOREVER);
		data_cnt++;
		//读取数据
		//mpu_read_n_reg(info, ACCEL_XOUT_H, (uint8_t*)info->sensor_data.raw_data, 14);
		
		//开中断
		gpio_pin_interrupt_configure_dt(info->int_gpio, 
		    GPIO_INT_EDGE_TO_ACTIVE);

		info->sensor_data.accel_x = sys_be16_to_cpu(info->sensor_data.raw_data[0]);
		info->sensor_data.accel_y = sys_be16_to_cpu(info->sensor_data.raw_data[1]);
		info->sensor_data.accel_z = sys_be16_to_cpu(info->sensor_data.raw_data[2]);
		info->sensor_data.temp = sys_be16_to_cpu(info->sensor_data.raw_data[3]);
		info->sensor_data.gyro_x = sys_be16_to_cpu(info->sensor_data.raw_data[4]);
		info->sensor_data.gyro_y = sys_be16_to_cpu(info->sensor_data.raw_data[5]);
		info->sensor_data.gyro_z = sys_be16_to_cpu(info->sensor_data.raw_data[6]);

		// for(int i=0; i<7; i++) printk("%x ", tmp[i]);	
		//printk("accel_z: %d\r\n",info->sensor_data.accel_z);
		
		// k_sleep(K_MSEC(10));
	}
}
#endif

/*
 * @description: mpu6050中断初始化 
 * @param:          无  
 * @return:         0		成功
 * 					ELSE	失败
 */
int mpu6050_init_interrupt(mpu6050_info_t *info)
{
    if(info->int_gpio == NULL || info->int_gpio->port == NULL){
        LOG_ERR("GPIO device is NULL!");
        return -ENODEV;
    }

	if (!gpio_is_ready_dt(info->int_gpio)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(info->int_gpio, GPIO_INPUT);

	gpio_init_callback(&info->gpio_cb,
			   mpu6050_gpio_callback,
			   BIT(info->int_gpio->pin));

	if (gpio_add_callback(info->int_gpio->port, &info->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

#if defined(MPU6050_TRIGGER_OWN_THREAD)
	k_sem_init(&info->gpio_sem, 0, 5);

	k_thread_create(&info->thread, info->thread_stack,
			MPU6050_THREAD_STACK_SIZE,
			mpu6050_thread, info,
			NULL, NULL, K_PRIO_COOP(MPU6050_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(MPU6050_TRIGGER_GLOBAL_THREAD)
	info->work.handler = mpu6050_work_cb;
#endif

	gpio_pin_interrupt_configure_dt(info->int_gpio,
					GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}


/*
 * @description: mpu6050初始化 
 * @param:          无  
 * @return:         无
 */
void mpu6050_init(void)
{
    //寄存器初始化
    int ret;
	int i=0;
	uint8_t tmp;

    mpu6050_dev.sensor_data.accel_sensitivity_shift = 14;   // 14 - bit(4,3)
    mpu6050_dev.sensor_data.gyro_sensitivity_x10 = mpu6050_gyro_sensitivity_x10[3];

	_i2c_io_init(&mpu6050_dev.soft_i2c_drv);

#ifdef  MPU6050_INT_ENABLE
    //中断配置
    ret = mpu6050_init_interrupt(&mpu6050_dev);
	if(ret) LOG_ERR("mpu6050_init_interrupt ERROR %X", ret);
#endif

	for (i=0; i < 11; i++){
        mpu_write_reg(&mpu6050_dev, mpu6050_init_cmd[i][0], mpu6050_init_cmd[i][1]);
        if (i == 0)
            k_sleep(K_MSEC(500));
    }
	
    for(i=0; i<11; i++){
        mpu_read_reg(&mpu6050_dev, mpu6050_init_cmd[i][0], &tmp);
        printk("tmp %02X\n\r", tmp);
    }
}




