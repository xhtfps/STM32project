/*
*********************************************************************************************************
*                                              Drive_PGA4311
* File			 : Drive_PGA4311.c
* By  			 : 李亮
* platform   	 : STM32F407ZG
* Data   		 : 2021/7
*********************************************************************************************************
*/
/* 头文件包含 ----------------------------------------------------------------*/
#include "Drive_PGA4311.h"


/**----------------------------------------------------------------------------
* 函 数 名	     : PGA4311_Init
* 功能说明       : PGA4311通信IO初始化
* 形    参       : 无
* 返 回 值		 ：无
-----------------------------------------------------------------------------*/
void PGA4311_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    GPIO_InitStructure.GPIO_Pin = PGA4311_CS | PGA4311_SCLK | PGA4311_SDO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	PGA4311_CS_1();
	PGA4311_SCLK_0();
	PGA4311_SDO_1();
}

/**----------------------------------------------------------------------------
* 函 数 名	     : PGA4311_SetGain
* 功能说明       : 设置PGA增益
* 形    参       : data：	4X8位的增益控制字
* 返 回 值		 ：无
-----------------------------------------------------------------------------*/
void PGA4311_SetGain(u32 data)
{
	u8 i; 
	u32 temp=0x80000000;

	PGA4311_CS_0();
	delay_us(5);
	for(i=0; i<32; i++)
	{
		PGA4311_SCLK_0();
		delay_us(3);
		if(data&temp){
			PGA4311_SDO_1();
		}else{
			PGA4311_SDO_0();
		}
		delay_us(3);
		PGA4311_SCLK_1();
		delay_us(5);
		temp = temp >> 1;
	}
	PGA4311_CS_1();
	PGA4311_SDO_1();
	PGA4311_SCLK_0();
	/* CS触发 */
	PGA4311_CS_0();
	delay_us(5);
	PGA4311_CS_1();
}


