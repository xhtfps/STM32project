#include "Drive_Communication.h"
#include "TFT_LCD.h"

/* 通信数据结构 */
communicationDataStruct communicationData = {{0},{0}};

// DDS结构体变量初始化
DDSDataStruct dds[2];

/**
* @brief  获取CRC校验值
* @param  none
* @retval CRC校验值
*/
static u16 crc_16(u8 *Datas, u16 Length)
{
	u8 j;
	u16 temp = 0xFFFF, i;

	for(i=0; i<Length; ++i)
	{
		temp ^= Datas[i];

		for(j=0; j<8; ++j)
		{
			if(temp & 1)
			{
				temp >>= 1;
				temp ^= 0xA001;
			}
			else
			{
				temp >>= 1;
			}
		}
	}

	return temp;
}

/**
 * @brief  USART1发送数据
 * @param  *sendData : 待发送的数据
 * @param  length    : 数据长度
 * @retval 无
 */
static void usartSendData(u8 *sendData,u16 length)
{
	u16 i;

	for(i=0 ; i<length ; i++)
	{
		USART_SendData(USART1,sendData[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // 等待发送缓冲区为空
	}
}

/**
* @brief  发送波形数据
* @param  data : DDS结构体变量
* @param  ch   : 通道号
* @retval none
*/
void sendData(DDSDataStruct data,u8 ch)
{
		u16 crcValue;
		u32 range=(u32)(data.range*1000);
		u32 phase=(u32)(data.phase*100);

		communicationData.txBuffer[0]  = 0xAA;        // 帧头
		communicationData.txBuffer[1]  = 0xA5;

		communicationData.txBuffer[2]  = (u8) data.mode >>8;      // 模式
		communicationData.txBuffer[3]  = (u8)(data.mode &0xff);

		communicationData.txBuffer[4]	 = ch;           // 通道
		communicationData.txBuffer[5]  = data.output;   // 输出开关

		communicationData.txBuffer[6]	 = (u8)(data.fre >>24);       // 频率
		communicationData.txBuffer[7]  = (u8)((data.fre>>16) & 0xff);
		communicationData.txBuffer[8]  = (u8)((data.fre>>8) & 0xff);
		communicationData.txBuffer[9]  = (u8)(data.fre & 0xff);

		communicationData.txBuffer[10] = (u8)(range>>24);           // 幅度
		communicationData.txBuffer[11] = (u8)((range>>16) & 0xff);
		communicationData.txBuffer[12] = (u8)((range>>8) & 0xff);
		communicationData.txBuffer[13] = (u8)(range & 0xff);

		communicationData.txBuffer[14] = (u8)(phase>>24);          // 相位
		communicationData.txBuffer[15] = (u8)((phase>>16) & 0xff);
		communicationData.txBuffer[16] = (u8)((phase>>8) & 0xff);
		communicationData.txBuffer[17] = (u8)(phase & 0xff);

		communicationData.txBuffer[18] = (u8)(data.step>>24);       // 步进
		communicationData.txBuffer[19] = (u8)((data.step>>16) & 0xff);
		communicationData.txBuffer[20] = (u8)((data.step>>8) & 0xff);
		communicationData.txBuffer[21] = (u8)(data.step & 0xff);

		communicationData.txBuffer[22] = (u8)(data.step_time>>24);  // 步进时间
		communicationData.txBuffer[23] = (u8)((data.step_time>>16) & 0xff);
		communicationData.txBuffer[24] = (u8)((data.step_time>>8) & 0xff);
		communicationData.txBuffer[25] = (u8)(data.step_time & 0xff);

		communicationData.txBuffer[26] = (u8)(data.fre_start>>24);  // 起始频率
		communicationData.txBuffer[27] = (u8)((data.fre_start>>16) & 0xff);
		communicationData.txBuffer[28] = (u8)((data.fre_start>>8) & 0xff);
		communicationData.txBuffer[29] = (u8)(data.fre_start & 0xff);

		communicationData.txBuffer[30] = (u8)(data.fre_stop>>24);   // 终止频率
		communicationData.txBuffer[31] = (u8)((data.fre_stop>>16) & 0xff);
		communicationData.txBuffer[32] = (u8)((data.fre_stop>>8) & 0xff);
		communicationData.txBuffer[33] = (u8)(data.fre_stop & 0xff);

		// 生成CRC校验值
		crcValue = crc_16(&communicationData.txBuffer[0],TX_LENGTH - 3);

		communicationData.txBuffer[TX_LENGTH-3] = crcValue>>8;
		communicationData.txBuffer[TX_LENGTH-2] = crcValue & 0xff;

		communicationData.txBuffer[TX_LENGTH-1] = 0x55;     // 帧尾

		usartSendData(communicationData.txBuffer,TX_LENGTH); // 传入要发送数组和数据长度
}

void Init_Uart(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);  // 使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); // 使能USART1时钟

	// 引脚复用配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); // PB6复用为USART1_TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); // PB7复用为USART1_RX

	// USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	     // 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // 上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);

    // USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;                    // 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // 字长8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;        // 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;           // 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE); // 使能串口1
}

void DDSDataInit(void)
{
	  /* 输出幅度 2V */
    dds[0].range = dds[1].range = 0.2f;

	  /* 输出频率 1000Hz */
	  dds[0].fre = dds[1].fre = 1000;

	  /* 输出相位 0° */
	  dds[0].phase = 0;
		dds[1].phase = 110;

	  /* 扫频步进 500Hz */
	  dds[0].step = dds[1].step = 500;

	  /* 扫频时间 10000us */
	  dds[0].step_time = dds[1].step_time = 10000;

	  /* 扫频起始频率 10000Hz */
		dds[0].fre_start = dds[1].fre_start = 10000;

	  /* 扫频终止频率 100000Hz */
	  dds[0].fre_stop = dds[1].fre_stop = 100000;

	  /* 默认为普通输出模式 */
	  dds[0].mode = dds[1].mode = NORMAL;

	  /* 默认打开输出 */
	  dds[0].output = 1;
		dds[1].output = 1;

	  sendData(dds[0],0);
		delay_ms(100);
    sendData(dds[1],1);
}
