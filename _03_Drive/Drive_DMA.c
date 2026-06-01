/*
*********************************************************************************************************
                                               _04_OS
    File			 : Drive_DMA.c
    By  			 :
    platform   : STM32F407ZG
	Data   		 : 2018/7/16
    function 	 : DMA配置程序
*********************************************************************************************************
*/
#include "Drive_DMA.h"
#define  ADC_PRECISION 			ADC_Resolution_12b

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有（静态）函数声明 ------------------------------------------------------*/

/* 全局变量定义 --------------------------------------------------------------*/
u16  ADC1_DMA2_Buff[ADC1_DMA2_LENTH] = {0};
u16  ADC3_DMA2_Buff[ADC3_DMA2_LENTH] = {0};
u16  DAC1_DMA1_Buff[DAC1_DMA1_LENTH] = {0};

/* 全局函数编写 --------------------------------------------------------------*/
/** ----------------------------------------------------------------------------
    @FunctionName  : ADC1_DMA2_ReLoad()
    @Description   : None
    @Data          : 2016/5/24
    @Explain       : Speed: 0 ~ 7 共8个档位，数值越小采集速度越快
    ------------------------------------------------------------------------------*/
void ADC1_DMA2_Reload(u8 Speed,uint32_t ADC_PreDiv)
{
	uint32_t ADC_SampleTime = ADC_SampleTime_3Cycles;//adc采样时间
	
	DMA_InitTypeDef 	  DMA_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;//配置ADC需要两个结构体
	ADC_InitTypeDef       ADC_InitStructure;
	GPIO_InitTypeDef  	  GPIO_InitStructure;
	
	switch(Speed)//控制ADC转换的时钟周期
	{
		case 0:
			ADC_SampleTime = ADC_SampleTime_3Cycles;
			break;
		case 1:
			ADC_SampleTime = ADC_SampleTime_15Cycles;
			break;
		case 2:
			ADC_SampleTime = ADC_SampleTime_28Cycles;
			break;
		case 3:
			ADC_SampleTime = ADC_SampleTime_56Cycles;
			break;
		case 4:
			ADC_SampleTime = ADC_SampleTime_84Cycles;
			break;
		case 5:
			ADC_SampleTime = ADC_SampleTime_112Cycles;
			break;
		case 6:
			ADC_SampleTime = ADC_SampleTime_144Cycles;
			break;
		case 7:
			ADC_SampleTime = ADC_SampleTime_480Cycles;
			break;
		default :
			ADC_SampleTime = ADC_SampleTime_3Cycles;
			break;;
	}
	
	/* DMA配置 ------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能
	DMA_DeInit(DMA2_Stream0);//恢复缺省配置
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE) {} //等待DMA可配置，等待DMA的数据流传输完成
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC1_DMA2_Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
	DMA_InitStructure.DMA_BufferSize = ADC1_DMA2_LENTH;//数据传输量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度:16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用循环模式 DMA_Mode_Normal DMA_Mode_Circular
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);//初始化DMA Stream
	DMA_SetCurrDataCounter(DMA2_Stream0,ADC1_DMA2_LENTH);//数据传输量
	DMA_Cmd(DMA2_Stream0, ENABLE);                      //开启DMA传输
	/* GPIO配置 -----------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PA1 通道1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	/* ADC配置 ------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //使能ADC1时钟
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);  //复位结束
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //禁用DMA对于ADC的访问
	ADC_CommonInitStructure.ADC_Prescaler = ADC_PreDiv;//ADC_Prescaler_Div8;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
	ADC_InitStructure.ADC_Resolution = ADC_PRECISION;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//非扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//定时器触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;  //
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime);//配置具体的ADC转换通道
	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);//ADC1的最后一次DMA传输完成之后，自动停止DMA请求
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);//开启AD转换器
	ADC_SoftwareStartConv(ADC1); //软件启动采集
}

/** ----------------------------------------------------------------------------
    @FunctionName  : ADC3_DMA2_Init()
    @Description   : None
    @Data          : 2016/8/16
    @Explain       : Speed: 0 ~ 7 共8个档位，数值越小采集速度越快
    ------------------------------------------------------------------------------*/
void ADC3_DMA2_Init(void)
{
	uint32_t ADC_SampleTime = ADC_SampleTime_3Cycles;
	
	DMA_InitTypeDef 	  DMA_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	GPIO_InitTypeDef  	  GPIO_InitStructure;
	
	/* DMA配置 ------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能
	DMA_DeInit(DMA2_Stream1);//恢复缺省配置
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE) {} //等待DMA可配置，数据流传输完成
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC3->DR;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC3_DMA2_Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
	DMA_InitStructure.DMA_BufferSize = ADC3_DMA2_LENTH;//数据传输量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度:16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用循环模式 DMA_Mode_Normal DMA_Mode_Circular
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream1,&DMA_InitStructure);//初始化DMA Stream
	DMA_SetCurrDataCounter(DMA2_Stream1,ADC3_DMA2_LENTH);//数据传输量
	DMA_Cmd(DMA2_Stream1, ENABLE);                      //开启DMA传输
		
	/* GPIO配置 -----------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//通道3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	/* ADC配置 ------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE); //使能ADC时钟
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);	  //ADC复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);  //复位结束
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //_DMA
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	ADC_InitStructure.ADC_Resolution = ADC_PRECISION;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1
	ADC_Init(ADC3, &ADC_InitStructure);//ADC初始化
	
	ADC_RegularChannelConfig(ADC3,ADC_Channel_3,1,ADC_SampleTime);
	ADC_DMARequestAfterLastTransferCmd(ADC3,ENABLE);
	ADC_DMACmd(ADC3,ENABLE);
	ADC_Cmd(ADC3,ENABLE);//开启AD转换器
	ADC_SoftwareStartConv(ADC3); //软件启动采集
}



/** ----------------------------------------------------------------------------
    @FunctionName  : ADC3_DMA2_Reload()
    @Description   : None
    @Data          : 2016/8/16
    @Explain       : Speed: 0 ~ 7 共8个档位，数值越小采集速度越快
    ------------------------------------------------------------------------------*/
void ADC3_DMA2_Reload(u8 Speed)
{
	uint32_t ADC_SampleTime = ADC_SampleTime_3Cycles;
	
	DMA_InitTypeDef 	  DMA_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	switch(Speed)
	{
		case 0:
			ADC_SampleTime = ADC_SampleTime_3Cycles;
			break;
		case 1:
			ADC_SampleTime = ADC_SampleTime_15Cycles;
			break;
		case 2:
			ADC_SampleTime = ADC_SampleTime_28Cycles;
			break;
		case 3:
			ADC_SampleTime = ADC_SampleTime_56Cycles;
			break;
		case 4:
			ADC_SampleTime = ADC_SampleTime_84Cycles;
			break;
		case 5:
			ADC_SampleTime = ADC_SampleTime_112Cycles;
			break;
		case 6:
			ADC_SampleTime = ADC_SampleTime_144Cycles;
			break;
		case 7:
			ADC_SampleTime = ADC_SampleTime_480Cycles;
			break;
		default :
			ADC_SampleTime = ADC_SampleTime_3Cycles;
			break;;
	}
	/* DMA配置 ------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能
	DMA_DeInit(DMA2_Stream1);
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE) {} //等待DMA可配置
		
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC3->DR;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC3_DMA2_Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
	DMA_InitStructure.DMA_BufferSize = ADC3_DMA2_LENTH;//数据传输量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度:16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用循环模式 DMA_Mode_Normal DMA_Mode_Circular
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream1,&DMA_InitStructure);//初始化DMA Stream
	DMA_SetCurrDataCounter(DMA2_Stream1,ADC3_DMA2_LENTH);//数据传输量
	DMA_Cmd(DMA2_Stream1, ENABLE);                      //开启DMA传输
//	/* GPIO配置 -----------------------------------*/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA时钟
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//通道3
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	/* ADC配置 ------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE); //使能ADC时钟
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);	  //ADC复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);  //复位结束
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	ADC_InitStructure.ADC_Resolution = ADC_PRECISION;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1
	ADC_Init(ADC3, &ADC_InitStructure);//ADC初始化
	
	ADC_RegularChannelConfig(ADC3,ADC_Channel_3,1,ADC_SampleTime);
	ADC_DMARequestAfterLastTransferCmd(ADC3,ENABLE);
	ADC_DMACmd(ADC3,ENABLE);
	ADC_Cmd(ADC3,ENABLE);//开启AD转换器
	ADC_SoftwareStartConv(ADC3); //软件启动采集
}

/** ----------------------------------------------------------------------------
    @FunctionName  : DMA1_Init()
    @Description   : None
    @Data          : 2016/7/15
    @Explain       : None
    ------------------------------------------------------------------------------*/
void DAC1_DMA1_Init(void)
{
	DMA_InitTypeDef 	  DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
	//DAC1使用DMA1的数据流5当中的通道7 
	
	/* DMA配置 ------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA2时钟使能
	DMA_DeInit(DMA1_Stream5);
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE) {} //等待DMA可配置
		
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&DAC ->DHR12R1;//DMA外设地址  DAC1
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)DAC1_DMA1_Buff;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//
	DMA_InitStructure.DMA_BufferSize = DAC1_DMA1_LENTH;//数据传输量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度:16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式 DMA_Mode_Normal DMA_Mode_Circular
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//中等优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);//初始化DMA Stream
	DMA_SetCurrDataCounter(DMA1_Stream5,DAC1_DMA1_LENTH);//数据传输量
	DMA_Cmd(DMA1_Stream5, ENABLE);//开启DMA传输
		
	/* DAC配置 ------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//①使能 PA 时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//②使能 DAC 时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//①初始化 GPIO
	DAC_InitType.DAC_Trigger=DAC_Trigger_T6_TRGO; //不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0; //屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;//输出缓存关闭
	DAC_Init(DAC_Channel_1,&DAC_InitType); //③初始化 DAC 通道 1
	DAC_SetChannel1Data(DAC_Align_12b_R,0);  //⑤12 位右对齐数据格式
	DAC_DMACmd(DAC_Channel_1,ENABLE);
	DAC_Cmd(DAC_Channel_1, ENABLE); //④使能 DAC 通道 1
	DAC1_DMA1_Reload(100);
}


//配置定时器6触发DAC1，65535的整数倍做预分频，取余部分作为自动重装值
void DAC1_DMA1_Reload(u32 speed)
{
	u16 speed0 = 1;
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	if(speed == 0)
		return;
	
	//定时器触发的设置，
	
	if(speed > 0xffff)//65535
	{
		speed0 = speed/0xffff;
		speed = speed%0xffff;
	}
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = speed;
	TIM_TimeBaseStructure.TIM_Prescaler = speed0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	TIM_Cmd(TIM6, ENABLE);
}


/*******************************(C) COPYRIGHT 2016 Wind（谢玉伸）*********************************/





