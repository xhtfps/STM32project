#ifndef USER_PGA112_H
#define USER_PGA112_H

#include "User_header.h"

#define PGA112_CMD_WRITE            0x2A00U
#define PGA112_CMD_READ             0x6A00U
#define PGA112_CMD_NOP              0x0000U
#define PGA112_CMD_SDN_DIS          0xE100U
#define PGA112_CMD_SDN_EN           0xE1F1U

#define PGA112_CHANNEL_CH0          0x00U
#define PGA112_CHANNEL_CH1          0x01U
#define PGA112_CHANNEL_CAL1         0x0CU
#define PGA112_CHANNEL_CAL2         0x0DU
#define PGA112_CHANNEL_CAL3         0x0EU
#define PGA112_CHANNEL_CAL4         0x0FU

#define PGA112_GAIN_1               0x00U
#define PGA112_GAIN_2               0x01U
#define PGA112_GAIN_4               0x02U
#define PGA112_GAIN_8               0x03U
#define PGA112_GAIN_16              0x04U
#define PGA112_GAIN_32              0x05U
#define PGA112_GAIN_64              0x06U
#define PGA112_GAIN_128             0x07U

#define PGA112_ULTRASONIC_CHANNEL   PGA112_CHANNEL_CH1
#define PGA112_DEFAULT_GAIN_CODE    PGA112_GAIN_8

void PGA112_Init(void);
void PGA112_Shutdown(FunctionalState new_state);
void PGA112_WriteConfig(uint8_t gain_code, uint8_t channel_code);
void PGA112_SetGainCode(uint8_t gain_code);
void PGA112_SelectChannel(uint8_t channel_code);
uint8_t PGA112_GetGainCode(void);
uint8_t PGA112_GetChannelCode(void);
uint16_t PGA112_GetGainValue(uint8_t gain_code);

#endif
