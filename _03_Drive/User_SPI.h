#ifndef USER_SPI_H
#define USER_SPI_H

#include "User_header.h"

#define USER_SPI_GPIO_CLK            RCC_AHB1Periph_GPIOE
#define USER_SPI_PORT                GPIOE

#define USER_SPI_CS_PIN              GPIO_Pin_0
#define USER_SPI_SCK_PIN             GPIO_Pin_2
#define USER_SPI_DIO_PIN             GPIO_Pin_4

#define USER_SPI_CS_LOW()            GPIO_ResetBits(USER_SPI_PORT, USER_SPI_CS_PIN)
#define USER_SPI_CS_HIGH()           GPIO_SetBits(USER_SPI_PORT, USER_SPI_CS_PIN)
#define USER_SPI_SCK_LOW()           GPIO_ResetBits(USER_SPI_PORT, USER_SPI_SCK_PIN)
#define USER_SPI_SCK_HIGH()          GPIO_SetBits(USER_SPI_PORT, USER_SPI_SCK_PIN)
#define USER_SPI_DIO_LOW()           GPIO_ResetBits(USER_SPI_PORT, USER_SPI_DIO_PIN)
#define USER_SPI_DIO_HIGH()          GPIO_SetBits(USER_SPI_PORT, USER_SPI_DIO_PIN)
#define USER_SPI_READ_DIO()          GPIO_ReadInputDataBit(USER_SPI_PORT, USER_SPI_DIO_PIN)

void User_SPI_Init(void);
void SPI_GPIO_Init(uint8_t mode);
void User_SPI_SetDioOutput(void);
void User_SPI_SetDioInput(void);
void User_SPI_StartFrame(void);
void User_SPI_EndFrame(void);
void User_SPI_SendData(uint32_t tx_data, uint8_t length, uint8_t pha);
uint16_t User_SPI_Transfer16(uint16_t tx_data);
void User_SPI_Write16(uint16_t tx_data);

#endif
