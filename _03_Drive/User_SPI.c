#include "User_SPI.h"

static void User_SPI_SetIdle(void);
static void User_SPI_ShiftBits(uint32_t tx_data, uint8_t length, uint8_t pha);

void User_SPI_Init(void)
{
    SPI_GPIO_Init(1U);
    User_SPI_SetIdle();
}

void SPI_GPIO_Init(uint8_t mode)
{
    GPIO_InitTypeDef gpio_init;
    (void)mode;

    RCC_AHB1PeriphClockCmd(USER_SPI_GPIO_CLK, ENABLE);

    gpio_init.GPIO_Pin = USER_SPI_CS_PIN | USER_SPI_SCK_PIN | USER_SPI_DIO_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP; 
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(USER_SPI_PORT, &gpio_init);

    User_SPI_SetIdle();
}

void User_SPI_StartFrame(void)
{
    User_SPI_SetDioOutput();
    USER_SPI_CS_LOW();
    delay_us(1);
}

void User_SPI_EndFrame(void)
{
    delay_us(1);
    USER_SPI_CS_HIGH();
    User_SPI_SetIdle();
}

void User_SPI_SendData(uint32_t tx_data, uint8_t length, uint8_t pha)
{
    User_SPI_StartFrame();
    User_SPI_ShiftBits(tx_data, length, pha);
    User_SPI_EndFrame();
}

uint16_t User_SPI_Transfer16(uint16_t tx_data)
{
    int8_t bit_index;
    uint16_t rx_data = 0U;

    for(bit_index = 15; bit_index >= 0; bit_index--)
    {
        if(((tx_data >> bit_index) & 0x01U) != 0U)
        {
            USER_SPI_DIO_HIGH();
        }
        else
        {
            USER_SPI_DIO_LOW();
        }

        delay_us(1);
        USER_SPI_SCK_HIGH();
        rx_data <<= 1;
        if(USER_SPI_READ_DIO() != Bit_RESET)
        {
            rx_data |= 0x01U;
        }
        delay_us(1);
        USER_SPI_SCK_LOW();
    }

    return rx_data;
}

void User_SPI_Write16(uint16_t tx_data)
{
    User_SPI_StartFrame();
    (void)User_SPI_Transfer16(tx_data);
    User_SPI_EndFrame();
}

static void User_SPI_SetIdle(void)
{
    USER_SPI_CS_HIGH();
    USER_SPI_SCK_LOW();
    USER_SPI_DIO_LOW();
    User_SPI_SetDioOutput();
}

void User_SPI_SetDioOutput(void)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.GPIO_Pin = USER_SPI_DIO_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(USER_SPI_PORT, &gpio_init);
}

void User_SPI_SetDioInput(void)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.GPIO_Pin = USER_SPI_DIO_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(USER_SPI_PORT, &gpio_init);
}

static void User_SPI_ShiftBits(uint32_t tx_data, uint8_t length, uint8_t pha)
{
    int16_t bit_index;

    if((length == 0U) || (length > 32U))
    {
        return;
    }

    for(bit_index = (int16_t)length - 1; bit_index >= 0; bit_index--)
    {
        if(pha == 0U)
        {
            USER_SPI_SCK_HIGH();
        }
        else
        {
            USER_SPI_SCK_LOW();
        }

        if(((tx_data >> bit_index) & 0x01U) != 0U)
        {
            USER_SPI_DIO_HIGH();
        }
        else
        {
            USER_SPI_DIO_LOW();
        }

        delay_us(1);

        if(pha == 0U)
        {
            USER_SPI_SCK_LOW();
        }
        else
        {
            USER_SPI_SCK_HIGH();
        }

        delay_us(1);
    }
}
