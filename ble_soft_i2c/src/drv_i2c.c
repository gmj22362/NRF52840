#include "drv_i2c.h"

/*
 * @description: i2c io初始化
 * @param:          drv  i2c_drv_t结构体
 * @return:         无
 */
void _i2c_io_init(i2c_drv_t* drv)
{
    //配置GPIO
    nrf_gpio_cfg(drv->sda_pin,                     \
        NRF_GPIO_PIN_DIR_OUTPUT,    \
        NRF_GPIO_PIN_INPUT_CONNECT, \
        NRF_GPIO_PIN_PULLUP,        \
        NRF_GPIO_PIN_S0S1,          \
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(drv->scl_pin,                     \
        NRF_GPIO_PIN_DIR_OUTPUT,    \
        NRF_GPIO_PIN_INPUT_CONNECT, \
        NRF_GPIO_PIN_PULLUP,        \
        NRF_GPIO_PIN_S0S1,          \
        NRF_GPIO_PIN_NOSENSE);
}

/*
 * @description: SDA IO模式切换
 * @param:          drv  i2c_drv_t结构体
*					is_output 是否为输出
 * @return:         无
 */
void _iic_sda_mode(i2c_drv_t *drv, bool is_output)
{
	if(is_output)	{
        nrf_gpio_cfg(drv->sda_pin,                     \
            NRF_GPIO_PIN_DIR_OUTPUT,    \
            NRF_GPIO_PIN_INPUT_CONNECT, \
            NRF_GPIO_PIN_PULLUP,        \
            NRF_GPIO_PIN_S0S1,          \
            NRF_GPIO_PIN_NOSENSE);
    }
	else //nrf_gpio_cfg_input(drv->sda_pin, NRF_GPIO_PIN_NOPULL);
    {
        nrf_gpio_cfg(drv->sda_pin,                     \
            NRF_GPIO_PIN_DIR_INPUT,    \
            NRF_GPIO_PIN_INPUT_CONNECT, \
            NRF_GPIO_PIN_PULLUP,        \
            NRF_GPIO_PIN_S0S1,          \
            NRF_GPIO_PIN_NOSENSE);
    }
}

/*
 * @description: i2c起始信号    SCL高电平时SDA产生下降沿
 * @param:          drv  i2c_drv_t结构体
 * @return:         无
 */
void _i2c_start_signal(i2c_drv_t* drv)
{
    _iic_sda_mode(drv, 1);
    SDA(1);
	SCL(1);                                 
	IIC_DELAY;      
	SDA(0);                 //拉低产生下降沿	                 
	IIC_DELAY;
    SCL(0);                                 
}

/*
 * @description: i2c停止信号    SCL高电平时SDA产生上升沿
 * @param:          drv  i2c_drv_t结构体
 * @return:         无
 */
void _i2c_stop_signal(i2c_drv_t* drv)
{
	_iic_sda_mode(drv, 1);
	SDA(0);
	SCL(1);                                 
	IIC_DELAY;      
	SDA(1);                 //拉高产生上升沿	                 
	IIC_DELAY;
    SCL(0);                                 
}

/*
 * @description: i2c发送字节    SCL高电平时发送
 * @param:          drv  i2c_drv_t结构体
 *                  txByte      发送的字节
 * @return:         uint8_t     应答
 */
uint8_t  _i2c_write_byte(i2c_drv_t* drv, uint8_t txByte)
{
    uint8_t ret=0;

	_iic_sda_mode(drv, 1);
	
    SCL(0);  		            					//拉低SCL, 此时可以改变数据
    IIC_DELAY;
    for (uint8_t mask = 0x80; mask > 0; mask >>= 1)                	
    {
        if ((mask & txByte) == 0)
            SDA(0);                					//发送0
        else
            SDA(1);                	 				//发送1

        IIC_DELAY;									
        SCL(1);                      				//拉高SCL一段时间，发送数据         
        IIC_DELAY;    
        SCL(0); 			            			//拉低SCL       		   	      
    }     

    IIC_DELAY;  
    SDA(1);
    SCL(1);                          				//拉高、释放总线
    IIC_DELAY;

    _iic_sda_mode(drv, 0);
    ret = nrf_gpio_pin_read(drv->sda_pin);                   //读到低电平为应答信号
    
    SCL(0); 
    IIC_DELAY; 
    return ret;                                               //接收到应答，返回1
}

/*
 * @description: i2c发送应答信号  SCL高电平期间，SDA低电平
 * @param:              drv  i2c_drv_t结构体
 * @return:             无
 */
void _i2c_ack(i2c_drv_t* drv)
{	
	SCL(0);
	SDA(0);
    IIC_DELAY; 
	SCL(1);
	IIC_DELAY; 
	SCL(0);
	IIC_DELAY;  
}

/*
 * @description: i2c发送非应答信号  SCL高电平期间，SDA高电平
 * @param:              drv  i2c_drv_t结构体
 * @return:             无
 */
void _i2c_nack(i2c_drv_t* drv)
{	
	SCL(0);
	SDA(1);
    IIC_DELAY; 
	SCL(1);
	IIC_DELAY; 
	SCL(0);
	IIC_DELAY; 
}

/*
 * @description: i2c读取字节    SCL高电平时读取
 * @param:          drv  i2c_drv_t结构体
 *                  ack      是否发送应答
 * @return:         uint8_t    读取到的字节
 */
uint8_t  _i2c_read_byte(i2c_drv_t* drv, uint8_t ack)
{
    uint8_t mask;
    uint8_t rx_byte = 0;

    SDA(1);
    _iic_sda_mode(drv, 0);                                //释放数据总线
                             
    SCL(0);											 //拉低SCL、使数据可变
	IIC_DELAY;                         
    for (mask = 0x80; mask > 0; mask >>= 1)                 //从高位开始读取数据          
    {                         
        SCL(1); 	                //拉高SCL           
        IIC_DELAY;                         

        if(nrf_gpio_pin_read(drv->sda_pin)){                       //读取数据
            rx_byte = rx_byte | mask;                     
        }

        SCL(0);                      //拉低SCL、使数据可变
        IIC_DELAY;  
    }
    
    _iic_sda_mode(drv, 1);  
    if(ack) _i2c_ack(drv);
    else _i2c_nack(drv);

    return rx_byte;
}


/*
 * @description: i2c写寄存器数据 
 * @param:          dev_addr    设备地址
 *                  reg_addr    寄存器地址  
 *                  data        数据缓存
 *                  data_len    数据长度
 * @return:         无
 */
void i2c_write_reg(i2c_drv_t* drv, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len)
{
    _i2c_start_signal(drv);
    _i2c_write_byte(drv, dev_addr<<1 | I2C_OPWR);
    _i2c_write_byte(drv, reg_addr);

    for(int i=0; i<data_len; i++){
        _i2c_write_byte(drv, data[i]);
    }
    _i2c_stop_signal(drv);
}

/*
 * @description: i2c读取寄存器数据 
 * @param:          dev_addr    设备地址
 *                  reg_addr    寄存器地址  
 *                  data        数据缓存
 *                  data_len    数据长度
 * @return:         无
 */
void i2c_read_reg(i2c_drv_t* drv, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len)
{
    _i2c_start_signal(drv);
    _i2c_write_byte(drv, dev_addr<<1 | I2C_OPWR);
    _i2c_write_byte(drv, reg_addr);

    _i2c_start_signal(drv);
    _i2c_write_byte(drv, dev_addr<<1 | I2C_OPRD);
    for(int i=0; i<data_len; i++){
        if(i == data_len-1)  *(data + i) = _i2c_read_byte(drv, 0);
        else *(data + i) = _i2c_read_byte(drv, 1);
    }
    _i2c_stop_signal(drv);
}
