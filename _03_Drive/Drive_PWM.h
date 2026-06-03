#ifndef __DRIVE_PWM_H
#define __DRIVE_PWM_H

/* 头文件包含 ----------------------------------------------------------------*/
#include "User_header.h"

/* 全局宏定义 ----------------------------------------------------------------*/

/***** 超声波测距 TIM1 引脚映射 *****/
#define ULTRASONIC_CH1_GPIO_PORT       GPIOA
#define ULTRASONIC_CH1_GPIO_PIN        GPIO_Pin_8
#define ULTRASONIC_CH1_GPIO_SOURCE     GPIO_PinSource8

#define ULTRASONIC_CH2_GPIO_PORT       GPIOA
#define ULTRASONIC_CH2_GPIO_PIN        GPIO_Pin_9
#define ULTRASONIC_CH2_GPIO_SOURCE     GPIO_PinSource9

#define ULTRASONIC_TIM1_AF             GPIO_AF_TIM1

/***** 超声波发射接收比较器控制引脚 *****/
#define ULTRASONIC_RX_CMP_CTRL_PORT    GPIOC
#define ULTRASONIC_RX_CMP_CTRL_PIN     GPIO_Pin_7
#define ULTRASONIC_TX_CMP_CTRL_PORT    GPIOC
#define ULTRASONIC_TX_CMP_CTRL_PIN     GPIO_Pin_6

#define ULTRASONIC_RX_CMP_ON()         GPIO_ResetBits(ULTRASONIC_RX_CMP_CTRL_PORT, ULTRASONIC_RX_CMP_CTRL_PIN)
#define ULTRASONIC_RX_CMP_OFF()        GPIO_SetBits(ULTRASONIC_RX_CMP_CTRL_PORT, ULTRASONIC_RX_CMP_CTRL_PIN)
#define ULTRASONIC_TX_CMP_ON()         GPIO_ResetBits(ULTRASONIC_TX_CMP_CTRL_PORT, ULTRASONIC_TX_CMP_CTRL_PIN)
#define ULTRASONIC_TX_CMP_OFF()        GPIO_SetBits(ULTRASONIC_TX_CMP_CTRL_PORT, ULTRASONIC_TX_CMP_CTRL_PIN)

/***** 超声波测距 PWM 时序参数 *****/
#define ULTRASONIC_PWM_FREQ_HZ         40000U
#define ULTRASONIC_TIM1_CLK_HZ         168000000U
#define ULTRASONIC_PWM_PERIOD_TICKS    (ULTRASONIC_TIM1_CLK_HZ / ULTRASONIC_PWM_FREQ_HZ)
#define ULTRASONIC_PWM_PULSE_TICKS     (ULTRASONIC_PWM_PERIOD_TICKS / 2U)
#define ULTRASONIC_BURST_CYCLES        8U
#define ULTRASONIC_RX_RECOVER_US       15U

/* 结构体声明 ----------------------------------------------------------------*/

/* 全局变量声明 --------------------------------------------------------------*/

/* 全局函数声明 --------------------------------------------------------------*/
void PWM1_Init(u16 arr,u16 psc);
void PWM1_CCR_Set(double xccr1);
void PWM2_Init(void);
void Ultrasonic_PWM_Init(void);
void Ultrasonic_FireBurst(void);

#endif
