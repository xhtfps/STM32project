/*
*********************************************************************************************************
                                               _04_OS
    File			 : Drive_PWM.c
    By  			 :
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
    function 	 : PWM配置程序
*********************************************************************************************************
*/
#include "Drive_PWM.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define PWM1_FREQUENCY 	10000    // PWM1输出的频率，单位Hz
#define PWM1_PERIOD 	(105000.0f/PWM1_FREQUENCY)

/* 私有（静态）函数声明 ------------------------------------------------------*/

/* 全局变量定义 --------------------------------------------------------------*/

/* 全局函数编写 --------------------------------------------------------------*/
/** ----------------------------------------------------------------------------
    @FunctionName  : PWM1_Init()
    @Description   : PWM1初始化
    @Data          : 2016/7/11
    @Explain       : 超声波测距相关PWM输出
    ------------------------------------------------------------------------------*/
void PWM1_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	GPIO_SetBits(GPIOA,GPIO_Pin_9);
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Prescaler = psc;   
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_Period = arr;    
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;     
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;                   
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;     
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE);  
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

/** ----------------------------------------------------------------------------
    @FunctionName  : PWM1_CCR_Set()
    @Description   : 设置PWM1占空比
    @Data          : 2016/7/11
    @Explain       : None
    ------------------------------------------------------------------------------*/
void PWM1_CCR_Set(double xccr1)
{
	xccr1 = xccr1*PWM1_PERIOD;
	TIM1->CCR1 = (u16)xccr1;
}

/** ----------------------------------------------------------------------------
    @FunctionName  : PWM2_Init()
    @Description   : PWM通道2初始化（输入捕获）
    @Data          : 2016/7/11
    @Explain       : PWM_Cap2捕获，PA1复用为定时器5
    ------------------------------------------------------------------------------*/
void PWM2_Init(void)
{
	TIM_ICInitTypeDef  TIM5_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
	
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=1000000-1;  
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; 
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
	TIM5_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC2,ENABLE);
	TIM_Cmd(TIM5,ENABLE ); 	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
}

/** ----------------------------------------------------------------------------
    @FunctionName  : Ultrasonic_PWM_Init()
    @Description   : 超声波测距TIM1 PWM初始化（CH1=PA8, CH2=PE11）
    @Data          : 2026/6/1
    ------------------------------------------------------------------------------*/
void Ultrasonic_PWM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef tim_oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_PinAFConfig(ULTRASONIC_CH1_GPIO_PORT, ULTRASONIC_CH1_GPIO_SOURCE, ULTRASONIC_TIM1_AF);
    GPIO_PinAFConfig(ULTRASONIC_CH2_GPIO_PORT, ULTRASONIC_CH2_GPIO_SOURCE, ULTRASONIC_TIM1_AF);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = ULTRASONIC_CH1_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ULTRASONIC_CH1_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = ULTRASONIC_CH2_GPIO_PIN;
    GPIO_Init(ULTRASONIC_CH2_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = ULTRASONIC_RX_CMP_CTRL_PIN | ULTRASONIC_TX_CMP_CTRL_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    // 8550高边开关低电平导通，默认关闭发射比较器，打开接收整形比较器
    ULTRASONIC_TX_CMP_OFF();
    ULTRASONIC_RX_CMP_ON();

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 0;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = ULTRASONIC_PWM_PERIOD_TICKS - 1U;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_RepetitionCounter = ULTRASONIC_BURST_CYCLES - 1U;
    TIM_TimeBaseInit(TIM1, &tim_base);

    TIM_OCStructInit(&tim_oc);
    tim_oc.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_OCPolarity = TIM_OCPolarity_High;
    tim_oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    tim_oc.TIM_Pulse = ULTRASONIC_PWM_PULSE_TICKS;
    TIM_OC1Init(TIM1, &tim_oc);

    tim_oc.TIM_OCMode = TIM_OCMode_PWM2;
    tim_oc.TIM_Pulse = ULTRASONIC_PWM_PULSE_TICKS;
    TIM_OC2Init(TIM1, &tim_oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Single);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, DISABLE);
}

/** ----------------------------------------------------------------------------
    @FunctionName  : Ultrasonic_FireBurst()
    @Description   : 触发一次超声波脉冲串（阻塞等待发送完成）
    @Data          : 2026/6/1
    ------------------------------------------------------------------------------*/
void Ultrasonic_FireBurst(void)
{
    // 发射期间关闭接收整形比较器，避免余震和串扰
    ULTRASONIC_RX_CMP_OFF();
    ULTRASONIC_TX_CMP_ON();

    TIM_SetCounter(TIM1, 0);
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_Cmd(TIM1, ENABLE);

    while(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) == RESET)
    {
    }

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_Cmd(TIM1, DISABLE);

    ULTRASONIC_TX_CMP_OFF();
    delay_us(ULTRASONIC_RX_RECOVER_US);
    ULTRASONIC_RX_CMP_ON();
}
