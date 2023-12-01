#include "i2c_soft.h"
#include "stm32f4xx_gpio.h"

#define DELAY_TIME	20


void SDA_Input_Mode()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(I2CPORT, &GPIO_InitStructure);
}


void SDA_Output_Mode()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(I2CPORT, &GPIO_InitStructure);
}


void SDA_Output( uint16_t val )
{
    if ( val )
    {
        I2CPORT->BSRR |= I2C_SDA;
    }
    else
    {
        I2CPORT->BSRR |= I2C_SDA<<16;
    }
}


void SCL_Output( uint16_t val )
{
    if ( val )
    {
        I2CPORT->BSRR |= I2C_SCL;
    }
    else
    {
        I2CPORT->BSRR|= I2C_SCL<<16;
    }
}


uint8_t SDA_Input(void)
{
	if(GPIO_ReadInputDataBit(I2CPORT, I2C_SDA) == Bit_SET){
		return 1;
	}else{
		return 0;
	}
}



static void delay1(unsigned int n)
{
    uint32_t i;
    for ( i = 0; i < n; ++i);
}


void I2CStart(void)
{
    SDA_Output_Mode();
    SDA_Output(1);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SDA_Output(0);
    delay1(DELAY_TIME);
    SCL_Output(0);
    delay1(DELAY_TIME);
}


void I2CStop(void)
{
    SDA_Output_Mode();
    SCL_Output(0);
    delay1(DELAY_TIME);
    SDA_Output(0);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SDA_Output(1);
    delay1(DELAY_TIME);

}


unsigned char I2CWaitAck(void)
{
    unsigned short cErrTime = 5;
    SDA_Input_Mode();
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    while(SDA_Input())
    {
        cErrTime--;
        delay1(DELAY_TIME);
        if (0 == cErrTime)
        {
            SDA_Output_Mode();
            I2CStop();
            return ERROR;
        }
    }
    SCL_Output(0);
    //  SDA_Output_Mode();
    delay1(DELAY_TIME);
    return SUCCESS;
}


void I2CSendAck(void)
{
    SDA_Output_Mode();
    SDA_Output(0);
    delay1(DELAY_TIME);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SCL_Output(0);
    delay1(DELAY_TIME);
    SDA_Output(1);

}


void I2CSendNotAck(void)
{
    // SDA_Output_Mode();
    SDA_Output(1);
    delay1(DELAY_TIME);
    delay1(DELAY_TIME);
    SCL_Output(1);
    delay1(DELAY_TIME);
    SCL_Output(0);
    delay1(DELAY_TIME);

}


void I2CSendByte(unsigned char cSendByte)
{
    SDA_Output_Mode();
    unsigned char  i = 8;
    while (i--)
    {
        SCL_Output(0);
        delay1(DELAY_TIME);
        SDA_Output(cSendByte & 0x80);
        delay1(DELAY_TIME);
        cSendByte += cSendByte;
        delay1(DELAY_TIME);
        SCL_Output(1);
        delay1(DELAY_TIME);
    }
    SCL_Output(0);
    delay1(DELAY_TIME);
}


unsigned char I2CReceiveByte(void)
{
    unsigned char i = 8;
    unsigned char cR_Byte = 0;
     SDA_Input_Mode();
    while (i--)
    {
        cR_Byte += cR_Byte;
        SCL_Output(0);
        delay1(DELAY_TIME);
        delay1(DELAY_TIME);
        SCL_Output(1);
        delay1(DELAY_TIME);
        cR_Byte |=  SDA_Input();
    }
    SCL_Output(0);
    delay1(DELAY_TIME);
     SDA_Output_Mode();
    return cR_Byte;
}

//
void I2CInit(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = I2C_SCL | I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(I2CPORT, &GPIO_InitStructure);
}
