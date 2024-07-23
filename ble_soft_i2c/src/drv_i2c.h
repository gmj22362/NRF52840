#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>


#define  NACK      0
#define  ACK       1

#define  LOW        0      //低
#define  HIGH       1      //高

#define  I2C_OPWR    0      //写
#define  I2C_OPRD    1      //读

#define SDA(x)    if(x) nrf_gpio_pin_set(drv->sda_pin);else nrf_gpio_pin_clear(drv->sda_pin)
#define SCL(x)    if(x) nrf_gpio_pin_set(drv->scl_pin);else nrf_gpio_pin_clear(drv->scl_pin)
#define IIC_DELAY	k_busy_wait(drv->clk_period)

//i2c驱动相关
typedef struct i2c_drv{
	char* name;					//名字
	uint32_t scl_pin;			//scl_pin
	uint32_t sda_pin;			//sda_pin
    unsigned int clk_period;    //时钟周期  单位us
}i2c_drv_t;

void _i2c_io_init(i2c_drv_t* drv);
uint8_t  _i2c_read_byte(i2c_drv_t* drv, uint8_t ack);
uint8_t  _i2c_write_byte(i2c_drv_t* drv, uint8_t txByte);
void _i2c_stop_signal(i2c_drv_t* drv);
void _i2c_start_signal(i2c_drv_t* drv);
void i2c_read_reg(i2c_drv_t* drv, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len);
void i2c_write_reg(i2c_drv_t* drv, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len);

 
#endif




