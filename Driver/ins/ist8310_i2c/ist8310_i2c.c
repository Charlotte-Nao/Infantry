#include "ist8310_i2c.h"

#include "cmsis_os.h"
#include "i2c.h"

#define IST8310_IIC_ADDRESS 0x0E  //the I2C address of IST8310

#define MAG_SEN 0.3f //raw int16 data change to uT unit. ԭʼ�������ݱ�� ��λut

#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE 0x10 //device ID

#define IST8310_WRITE_REG_NUM 4

//第一列:IST8310的寄存器
//第二列:需要写入的寄存器值
//第三列:返回的错误码
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] ={
    {0x0B, 0x08, 0x01},     //开启中断，并且设置低电平
    {0x41, 0x09, 0x02},     //平均采样两次
    {0x42, 0xC0, 0x03},     //必须是0xC0
    {0x0A, 0x0B, 0x04}};    //200Hz输出频率



uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    /*RESET*/
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    vTaskDelay(sleepTime);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    vTaskDelay(sleepTime);

    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, IST8310_WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&res,1,10);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }

    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, ist8310_write_reg_data_error[writeNum][0],I2C_MEMADD_SIZE_8BIT,&ist8310_write_reg_data_error[writeNum][1],1,10);
        vTaskDelay(wait_time);
        HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, ist8310_write_reg_data_error[writeNum][0],I2C_MEMADD_SIZE_8BIT,&res,1,10);
        vTaskDelay(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

void ist8310_read_mag(fp32 mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    //read the "DATAXL" register (0x03)
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, 0x03,I2C_MEMADD_SIZE_8BIT,buf,6,10);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
