#ifndef _MPU6050_H_
#define _MPU6050_H_
 
 
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
 
#include <zephyr/drivers/i2c.h>
 
#include <zephyr/sys/byteorder.h>
 
#include <zephyr/devicetree/gpio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
 
#include <stdio.h>
#include <zephyr/logging/log.h>
 
 #define MPU6050_REG_CHIP_ID		0x75
#define MPU6050_CHIP_ID			0x68
#define MPU9250_CHIP_ID			0x71
#define MPU6880_CHIP_ID			0x19

/* MPU6050内部寄存器地址 */
#define SMPLRT_DIV  0x19 //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG   0x1A //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG  0x1B //陀螺仪自检及测量范围，典型值：0xF8(不自检，+/-2000deg/s)
#define ACCEL_CONFIG 0x1C //加速计自检、测量范围，典型值：0x19(不自检，+/-G)
 
#define MPU6050_GYRO_FS_SHIFT		3
#define MPU6050_ACCEL_FS_SHIFT		3
 
#define FIFO_EN      0X23
#define INT_ENABLE   0X38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H  0x41
#define TEMP_OUT_L  0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define SIGNAL_PATH_RESET 0X68  //reset the analog and digital signal paths
#define USER_CTRL 0X6A  
#define PWR_MGMT_1  0x6B //电源管理，典型值：0x00(正常启用)
#define PWR_MGMT_2  0x6C
 
#define MPU_SENSOR_ADDR                 0X68        /*!< 7 bits Slave address of the SGM sensor -- ADDR connect to GND */
 
#define MPU_RDY_IO                      7   // 转换完成信号，可读ADC数据
 
#define MPU6050_INT_ENABLE
 
#ifdef  MPU6050_INT_ENABLE
#define MPU6050_THREAD_STACK_SIZE   1024*10
#define MPU6050_THREAD_PRIORITY     10
#define MPU6050_TRIGGER_OWN_THREAD
#endif
 
//mpu6050 采样数据结构体
typedef struct mpu6050_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;
 
	int16_t temp;
 
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;
}mpu6050_data_t;
 
//mpu6050 结构体
typedef struct mpu6050_info{
    struct i2c_dt_spec i2c;
    mpu6050_data_t sensor_data;
 
#ifdef MPU6050_INT_ENABLE
    struct gpio_dt_spec int_gpio;
	struct gpio_callback gpio_cb;
 
#if defined(MPU6050_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, MPU6050_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(MPU6050_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
 
#endif /* CONFIG_MPU6050_TRIGGER */
}mpu6050_info_t;
 
extern mpu6050_info_t mpu6050_dev;
 
int mpu6050_init(void);
void mpu_read_reg(mpu6050_info_t *dev, uint8_t reg_addr, uint8_t *data);
void mpu_write_reg(mpu6050_info_t *dev, uint8_t reg_addr, uint8_t data);
void mpu_read_n_reg(mpu6050_info_t *dev, uint8_t start_reg, uint8_t* data, uint32_t len);
int mpu6050_channel_get(mpu6050_info_t *info, enum sensor_channel chan, struct sensor_value *val);
 
#endif
