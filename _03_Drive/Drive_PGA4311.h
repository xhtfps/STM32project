/*
*********************************************************************************************************
*                                              Drive_PGA4311
* File			 : Drive_PGA4311.h
* By  			 : 李亮
* platform   	 : STM32F407ZG
* Data   		 : 2021/7
*********************************************************************************************************
*/
/* 防止重调用 ----------------------------------------------------------------*/
#ifndef _DRIVE_PGA4311_H_
#define _DRIVE_PGA4311_H_
/* 头文件包含 ----------------------------------------------------------------*/ 
#include "User_Header.h"
/* 通信宏定义 ----------------------------------------------------------------*/
#define		PGA4311_CS			GPIO_Pin_3
#define		PGA4311_SDO			GPIO_Pin_5
#define		PGA4311_SCLK		GPIO_Pin_7

#define 	PGA4311_CS_0()		GPIO_ResetBits(GPIOG, PGA4311_CS)
#define 	PGA4311_CS_1()		GPIO_SetBits(GPIOG, PGA4311_CS)
#define 	PGA4311_SCLK_0()	GPIO_ResetBits(GPIOG, PGA4311_SCLK)
#define 	PGA4311_SCLK_1()	GPIO_SetBits(GPIOG, PGA4311_SCLK)
#define 	PGA4311_SDO_0()		GPIO_ResetBits(GPIOG, PGA4311_SDO)
#define 	PGA4311_SDO_1()		GPIO_SetBits(GPIOG, PGA4311_SDO)

/* 全局函数编写 --------------------------------------------------------------*/
void PGA4311_Init();
void PGA4311_SetGain(u32 data);


#endif 

