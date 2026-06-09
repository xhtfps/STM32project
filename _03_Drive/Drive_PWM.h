#ifndef __DRIVE_PWM_H
#define __DRIVE_PWM_H

/* 头文件包含 ----------------------------------------------------------------*/
#include "User_header.h" // 包含工程基础定义（如u16, u32及硬件库等）

/* 全局宏定义 ----------------------------------------------------------------*/

/***** 超声波测距 TIM4 引脚映射 *****/
// 定义TIM4的两个输出通道，用于驱动超声波换能器
#define ULTRASONIC_CH1_GPIO_PORT       GPIOD
#define ULTRASONIC_CH1_GPIO_PIN        GPIO_Pin_12
#define ULTRASONIC_CH1_GPIO_SOURCE     GPIO_PinSource12 // 用于GPIO_PinAFConfig配置复用

#define ULTRASONIC_CH2_GPIO_PORT       GPIOD
#define ULTRASONIC_CH2_GPIO_PIN        GPIO_Pin_13
#define ULTRASONIC_CH2_GPIO_SOURCE     GPIO_PinSource13

#define ULTRASONIC_TIM_AF              GPIO_AF_TIM4    // 定义复用功能映射为TIM4

/***** 超声波发射接收比较器控制引脚 *****/
// 用于控制外部模拟前端电路（如通过三极管驱动开关电源）
// 假设电路逻辑为低电平导通(Active Low)，高电平关断
#define ULTRASONIC_RX_CMP_CTRL_PORT    GPIOC
#define ULTRASONIC_RX_CMP_CTRL_PIN     GPIO_Pin_7      // 接收整形电路开关
#define ULTRASONIC_TX_CMP_CTRL_PORT    GPIOC
#define ULTRASONIC_TX_CMP_CTRL_PIN     GPIO_Pin_6      // 发射驱动电路开关

// 开关控制宏，封装了GPIO操作，方便业务层直接调用
#define ULTRASONIC_RX_CMP_ON()         GPIO_ResetBits(ULTRASONIC_RX_CMP_CTRL_PORT, ULTRASONIC_RX_CMP_CTRL_PIN)
#define ULTRASONIC_RX_CMP_OFF()        GPIO_SetBits(ULTRASONIC_RX_CMP_CTRL_PORT, ULTRASONIC_RX_CMP_CTRL_PIN)
#define ULTRASONIC_TX_CMP_ON()         GPIO_ResetBits(ULTRASONIC_TX_CMP_CTRL_PORT, ULTRASONIC_TX_CMP_CTRL_PIN)
#define ULTRASONIC_TX_CMP_OFF()        GPIO_SetBits(ULTRASONIC_TX_CMP_CTRL_PORT, ULTRASONIC_TX_CMP_CTRL_PIN)

/***** 超声波测距 PWM 时序参数 *****/
#define ULTRASONIC_PWM_FREQ_HZ         40000U          // 目标PWM频率：40kHz（常用超声波探头中心频率）
#define ULTRASONIC_TIM_CLK_HZ          84000000U       // 定时器时钟频率（84MHz）
// 计算自动重装载值（ARR），决定一个完整周期的计数次数
#define ULTRASONIC_PWM_PERIOD_TICKS    (ULTRASONIC_TIM_CLK_HZ / ULTRASONIC_PWM_FREQ_HZ)
// 计算占空比计数值，设为周期的50%以获得对称方波
#define ULTRASONIC_PWM_PULSE_TICKS     (ULTRASONIC_PWM_PERIOD_TICKS / 2U)
// 单次发射的脉冲个数（Burst），脉冲越多能量越强，但近场盲区会变大
#define ULTRASONIC_BURST_CYCLES        8U              
// 接收恢复时间（微秒）：发射停止后，探头会有残余振动（余震），此时间用于等待探头静止，防止回波被误触发
#define ULTRASONIC_RX_RECOVER_US       15U             

/* 结构体声明 ----------------------------------------------------------------*/

/* 全局变量声明 --------------------------------------------------------------*/

/* 全局函数声明 --------------------------------------------------------------*/
// 初始化基础PWM通道
void PWM1_Init(u16 arr,u16 psc);
// 动态设置占空比
void PWM1_CCR_Set(double xccr1);
// 初始化输入捕获通道（用于记录回波时间）
void PWM2_Init(void);
// 超声波专用初始化函数（含模拟电路控制）
void Ultrasonic_PWM_Init(void);
// 触发一次超声波Burst发射
void Ultrasonic_FireBurst(void);
// 手动开启PWM输出（发射）
void Ultrasonic_PWM_OutputEnable(void);
// 手动关闭PWM输出（停止发射并进入接收模式）
void Ultrasonic_PWM_OutputDisable(void);

#endif
