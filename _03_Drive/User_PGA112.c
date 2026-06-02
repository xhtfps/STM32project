#include "User_PGA112.h"
#include "User_SPI.h"

static uint8_t g_pga112_gain_code = PGA112_DEFAULT_GAIN_CODE;
static uint8_t g_pga112_channel_code = PGA112_ULTRASONIC_CHANNEL;

static uint16_t PGA112_BuildWriteWord(uint8_t gain_code, uint8_t channel_code);

void PGA112_Init(void)
{
    delay_ms(2);
    PGA112_Shutdown(DISABLE);
    PGA112_WriteConfig(g_pga112_gain_code, g_pga112_channel_code);
}

void PGA112_Shutdown(FunctionalState new_state)
{
    if(new_state != DISABLE)
    {
        User_SPI_Write16(PGA112_CMD_SDN_EN);
    }
    else
    {
        User_SPI_Write16(PGA112_CMD_SDN_DIS);
    }
    delay_us(10);
}

void PGA112_WriteConfig(uint8_t gain_code, uint8_t channel_code)
{
    gain_code &= 0x07U;
    channel_code &= 0x0FU;

    g_pga112_gain_code = gain_code;
    g_pga112_channel_code = channel_code;

    User_SPI_Write16(PGA112_BuildWriteWord(gain_code, channel_code));
    delay_us(10);
}

void PGA112_SetGainCode(uint8_t gain_code)
{
    PGA112_WriteConfig(gain_code, g_pga112_channel_code);
}

void PGA112_SelectChannel(uint8_t channel_code)
{
    PGA112_WriteConfig(g_pga112_gain_code, channel_code);
}

uint8_t PGA112_GetGainCode(void)
{
    return g_pga112_gain_code;
}

uint8_t PGA112_GetChannelCode(void)
{
    return g_pga112_channel_code;
}

uint16_t PGA112_GetGainValue(uint8_t gain_code)
{
    return (uint16_t)(1U << (gain_code & 0x07U));
}

static uint16_t PGA112_BuildWriteWord(uint8_t gain_code, uint8_t channel_code)
{
    return (uint16_t)(PGA112_CMD_WRITE | ((uint16_t)(gain_code & 0x07U) << 4) |
                      (uint16_t)(channel_code & 0x0FU));
}
