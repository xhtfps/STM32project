/*
*********************************************************************************************************
                                               _04_OS
    File             : Drive_PWM.c
    By               :
    platform   : STM32F407ZG
    Data         : 2018/7/16
    function     : PWM配置程序及超声波底层驱动
*********************************************************************************************************
*/
#include "Drive_PWM.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define PWM1_FREQUENCY  10000    // PWM1输出的频率，单位Hz (10kHz)
// 根据定时器时钟和目标频率计算周期值 (注意：这里的105000.0f可能基于特定的系统时钟或预分频设置)
#define PWM1_PERIOD     (105000.0f/PWM1_FREQUENCY) 

/* 私有（静态）函数声明 ------------------------------------------------------*/

/* 全局变量定义 --------------------------------------------------------------*/

/* 全局函数编写 --------------------------------------------------------------*/
/** ----------------------------------------------------------------------------
    @FunctionName  : PWM1_Init()
    @Description   : PWM1初始化 (通用配置版)
    @Data          : 2016/7/11
    @Explain       : 早期版本的超声波测距相关PWM输出，使用TIM4的通道1(PD12)和通道2(PD13)
    ------------------------------------------------------------------------------*/
void PWM1_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    // 1. 开启时钟：TIM4挂载在APB1总线，GPIOD挂载在AHB1总线
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);     
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   
    
    // 2. 引脚复用配置：将PD12和PD13复用为TIM4的定时器引脚
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
    
    // 3. GPIO模式配置：复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 复用功能模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 输出速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出，增强驱动能力
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        // 不使用上下拉电阻
    GPIO_Init(GPIOD,&GPIO_InitStructure);
    // 默认输出高电平（在复用模式下，实际上由外设控制，此句影响不大）
    GPIO_SetBits(GPIOD,GPIO_Pin_12);
    GPIO_SetBits(GPIOD,GPIO_Pin_13);
    
    // 4. 定时器基础配置
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数模式
    TIM_TimeBaseStructure.TIM_Prescaler = psc;   // 预分频器值（决定计数频率）
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 高级定时器才用的重复计数器，通用定时器设0
    TIM_TimeBaseStructure.TIM_Period = arr;    // 自动重装载值（决定PWM周期）
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
    
    // 5. 定时器输出比较(OC)配置 - 通道1 (PD12)
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;     // 空闲状态为高电平
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1：计数值小于CCR时输出有效电平
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; // 有效电平为低电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 开启输出
    TIM_OCInitStructure.TIM_Pulse = 0;                   // 初始占空比值 (CCR寄存器的值)
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    
    // 6. 定时器输出比较(OC)配置 - 通道2 (PD13)
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;     
    // 注意：通道2使用PWM模式2，结合通道1的PWM模式1，在相同的CCR值下，两个通道会输出互补的波形（反相），适合差分驱动超声波探头
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    
    // 7. 使能预装载寄存器：修改CCR（占空比）时，会在下一个更新事件（周期结束）才生效，避免波形毛刺
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
    TIM_ARRPreloadConfig(TIM4,ENABLE); // 使能ARR的预装载
    
    // 8. 使能TIM4定时器开始工作
    TIM_Cmd(TIM4, ENABLE);  
}

/** ----------------------------------------------------------------------------
    @FunctionName  : PWM1_CCR_Set()
    @Description   : 设置PWM1占空比
    @Data          : 2016/7/11
    @Explain       : 传入0~1.0的浮点数，计算出CCR值并写入
    ------------------------------------------------------------------------------*/
void PWM1_CCR_Set(double xccr1)
{
    // 将0~1的小数比例换算为实际的计数值
    xccr1 = xccr1*PWM1_PERIOD;
    // 更新TIM4通道1的捕获/比较寄存器
    TIM4->CCR1 = (u16)xccr1;
}

/** ----------------------------------------------------------------------------
    @FunctionName  : PWM2_Init()
    @Description   : PWM通道2初始化（输入捕获配置）
    @Data          : 2016/7/11
    @Explain       : 配置TIM5_CH2 (PA1)，用于捕获超声波ECHO返回的脉宽时间
    ------------------------------------------------------------------------------*/
void PWM2_Init(void)
{
    TIM_ICInitTypeDef  TIM5_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 开启时钟：TIM5(APB1) 和 GPIOA(AHB1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);     
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    // 2. 引脚配置：PA1复用为输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // 下拉，防止未连接时受到干扰误触发
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    // 3. 复用映射：连接PA1到TIM5
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
    
    // 4. 定时器基础配置：用于计时脉冲宽度
    // 假设系统时钟设为168MHz，TIM5时钟=84M*2=168MHz。预分频设为168-1，则定时器频率为1MHz，即1微秒(us)计一次数。
    TIM_TimeBaseStructure.TIM_Prescaler = 168-1;  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseStructure.TIM_Period = 1000000-1;  // 周期设为最大，1000000us = 1秒溢出一次
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
    
    // 5. 输入捕获(IC)配置
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; // 使用通道2
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    // 初始配置为上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 引脚直接映射到通道，不交叉
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   // 捕获不分频：每次边沿都触发
    TIM5_ICInitStructure.TIM_ICFilter = 0x00; // 不使用滤波器
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);
    
    // 6. 中断配置：开启TIM5的更新中断（溢出）和通道2的捕获中断
    TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC2,ENABLE);
    TIM_Cmd(TIM5,ENABLE );  // 使能TIM5
    
    // 7. NVIC中断控制器配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; // 定时器5中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      // 子优先级 1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         // 开启中断通道
    NVIC_Init(&NVIC_InitStructure); 
}

/** ----------------------------------------------------------------------------
    @FunctionName  : Ultrasonic_PWM_Init()
    @Description   : 超声波测距 TIM4 PWM 初始化（CH1=PD12, CH2=PD13）优化版
    @Data          : 2026/6/1
    @Explain       : 包含收发切换控制引脚，硬件级防串扰与余震控制
    ------------------------------------------------------------------------------*/
void Ultrasonic_PWM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef tim_oc;

    // 1. 开启时钟：除了外设时钟，增加了GPIOC时钟（用于控制收发比较器）
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 2. 将驱动引脚复用至定时器AF功能（这里使用了宏定义，提升了代码移植性）
    GPIO_PinAFConfig(ULTRASONIC_CH1_GPIO_PORT, ULTRASONIC_CH1_GPIO_SOURCE, ULTRASONIC_TIM_AF);
    GPIO_PinAFConfig(ULTRASONIC_CH2_GPIO_PORT, ULTRASONIC_CH2_GPIO_SOURCE, ULTRASONIC_TIM_AF);

    // 3. 配置推挽输出GPIO引脚
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = ULTRASONIC_CH1_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz; // 提高翻转速率保证方波质量
    gpio.GPIO_OType = GPIO_OType_PP; // 推挽输出
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ULTRASONIC_CH1_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = ULTRASONIC_CH2_GPIO_PIN;
    GPIO_Init(ULTRASONIC_CH2_GPIO_PORT, &gpio);

    // 4. 配置GPIOC用于外围模拟电路的收发比较器电源使能引脚
    gpio.GPIO_Pin = ULTRASONIC_RX_CMP_CTRL_PIN | ULTRASONIC_TX_CMP_CTRL_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT; // 普通输出模式
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP; // 上拉，结合外部PNP三极管(8550)特性，默认输出高电平时为关断状态
    GPIO_Init(GPIOC, &gpio);

    // 5. 初始化模拟前端状态
    // 8550高边开关低电平导通：此处逻辑宏负责关闭发射端电源，打开接收端整形比较器电源，进入待命状态
    ULTRASONIC_TX_CMP_OFF();
    ULTRASONIC_RX_CMP_ON();

    // 6. 定时器频率与周期配置
    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 0; // 不分频，使用系统最高频率来保证PWM精细度
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = ULTRASONIC_PWM_PERIOD_TICKS - 1U; // 装载宏定义的40kHz超声波专用周期值
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tim_base);

    // 7. 定时器通道1配置 (PWM1模式)
    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode = TIM_OCMode_PWM1; 
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    tim_oc.TIM_OCIdleState = TIM_OCIdleState_Reset; // 空闲时为低电平（0V）
    tim_oc.TIM_Pulse = ULTRASONIC_PWM_PULSE_TICKS; // 设定占空比值 (通常是周期的50%)
    TIM_OC1Init(TIM4, &tim_oc);

    // 8. 定时器通道2配置 (PWM2模式) - 产生与通道1相位相差180度的反相波形，驱动推挽变压器或直接驱动探头两端
    tim_oc.TIM_OCMode = TIM_OCMode_PWM2;
    tim_oc.TIM_Pulse = ULTRASONIC_PWM_PULSE_TICKS;
    TIM_OC2Init(TIM4, &tim_oc);

    // 9. 开启预装载机制
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    
    // 10. 暂不使能定时器，等待手动触发发射
    TIM_Cmd(TIM4, DISABLE);
}

/** ----------------------------------------------------------------------------
    @FunctionName  : Ultrasonic_PWM_OutputEnable()
    @Description   : 持续发送超声波
    @Explain       : 配合外部逻辑手动起停。注意在发射时必须关断接收前端以防串扰
    ------------------------------------------------------------------------------*/
void Ultrasonic_PWM_OutputEnable(void)
{
    ULTRASONIC_RX_CMP_OFF(); // 关键动作：关闭接收电路，防止发射的强信号直接耦合进接收端导致"自激"或误判
    ULTRASONIC_TX_CMP_ON();  // 打开升压/发射电路电源
    
    TIM_SetCounter(TIM4, 0); // 清零计数器，保证从完整周期开始发送
    TIM_ClearFlag(TIM4, TIM_FLAG_Update); // 清除可能残留的更新标志位
    TIM_Cmd(TIM4, ENABLE);   // 启动PWM输出
}

/** ----------------------------------------------------------------------------
    @FunctionName  : Ultrasonic_PWM_OutputDisable()
    @Description   : 停止发送超声波并恢复接收
    @Explain       : 停止后必须有一个死区延时（余震消除），再打开接收电路
    ------------------------------------------------------------------------------*/
void Ultrasonic_PWM_OutputDisable(void)
{
    TIM_Cmd(TIM4, DISABLE);  // 立即关闭PWM输出
    ULTRASONIC_TX_CMP_OFF(); // 关断发射端电路电源
    
    // 硬件级盲区控制：压电陶瓷探头在停止激励后会有机械余震（Ring-down），
    // 延迟一段时间等待探头完全静止，避免余震被当成回波接收
    delay_us(ULTRASONIC_RX_RECOVER_US); 
    
    ULTRASONIC_RX_CMP_ON();  // 安全期过后，重新打开接收电路监听回波
}

/** ----------------------------------------------------------------------------
    @FunctionName  : Ultrasonic_FireBurst()
    @Description   : 触发一次超声波脉冲串（阻塞等待发送完成）
    @Data          : 2026/6/1
    @Explain       : 最常用的发射函数。发射固定数量（Burst）的40kHz方波后自动转入接收模式。
    ------------------------------------------------------------------------------*/
void Ultrasonic_FireBurst(void)
{
    // 1. 发射期间关闭接收整形比较器，避免余震和强驱动信号引发电气串扰（近场盲区的物理防御）
    ULTRASONIC_RX_CMP_OFF();
    ULTRASONIC_TX_CMP_ON();

    // 2. 状态重置与启动
    TIM_SetCounter(TIM4, 0);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_Cmd(TIM4, ENABLE); // 开始输出PWM脉冲
    
    // 3. 计算脉冲串耗时并进行阻塞延时
    // 耗时公式 = (1000000 / 频率) * 发射周期数 = 1个周期的微秒数 * N个周期
    // 例如：40kHz下1个周期是25us，发射8个波（BURST_CYCLES=8），则延时 200us
    delay_us((uint32_t)((1000000U / ULTRASONIC_PWM_FREQ_HZ) * ULTRASONIC_BURST_CYCLES));
    
    // 4. 延时结束，停止PWM波输出
    TIM_Cmd(TIM4, DISABLE);

    // 5. 关断发射电路，防止静态漏电或噪声干扰
    ULTRASONIC_TX_CMP_OFF();
    
    // 6. 探头机械振子停振缓冲延时 (防止余波误判)
    delay_us(ULTRASONIC_RX_RECOVER_US);
    
    // 7. 重新开启接收监听电路，等待回波信号
    ULTRASONIC_RX_CMP_ON();
}
