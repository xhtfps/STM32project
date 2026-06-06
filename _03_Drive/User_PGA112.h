#ifndef USER_PGA112_H
#define USER_PGA112_H

#include "User_header.h"

// PGA112寄存器命令和配置定义
#define PGA112_CMD_WRITE            0x2A00U
#define PGA112_CMD_READ             0x6A00U
#define PGA112_CMD_NOP              0x0000U
#define PGA112_CMD_SDN_DIS          0xE100U
#define PGA112_CMD_SDN_EN           0xE1F1U

// PGA112输入通道定义
#define PGA112_CHANNEL_CH0          0x00U
#define PGA112_CHANNEL_CH1          0x01U
#define PGA112_CHANNEL_CAL1         0x0CU
#define PGA112_CHANNEL_CAL2         0x0DU
#define PGA112_CHANNEL_CAL3         0x0EU
#define PGA112_CHANNEL_CAL4         0x0FU

// PGA112增益代码定义，对应增益倍数1x到128x
#define PGA112_GAIN_1               0x00U
#define PGA112_GAIN_2               0x01U
#define PGA112_GAIN_4               0x02U
#define PGA112_GAIN_8               0x03U
#define PGA112_GAIN_16              0x04U
#define PGA112_GAIN_32              0x05U
#define PGA112_GAIN_64              0x06U
#define PGA112_GAIN_128             0x07U

#define PGA112_ULTRASONIC_CHANNEL   PGA112_CHANNEL_CH0
#define PGA112_DEFAULT_GAIN_CODE    PGA112_GAIN_8

void PGA112_Init(void);                                             // 初始化PGA112，设置默认增益和通道
void PGA112_Shutdown(FunctionalState new_state);                    // 控制PGA112的电源状态
void PGA112_WriteConfig(uint8_t gain_code, uint8_t channel_code);   // 写入PGA112配置寄存器，设置增益和输入通道
void PGA112_SetGainCode(uint8_t gain_code);                         // 设置PGA增益代码
void PGA112_SelectChannel(uint8_t channel_code);                    // 设置PGA输入通道
uint8_t PGA112_GetGainCode(void);                                   // 获取当前设置的PGA增益代码
uint8_t PGA112_GetChannelCode(void);                                // 获取当前设置的PGA输入通道代码
uint16_t PGA112_GetGainValue(uint8_t gain_code);                    // 根据增益代码获取对应的增益倍数

#endif
