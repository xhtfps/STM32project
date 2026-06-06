#include "User_PGA112.h"
#include "User_SPI.h"

static uint8_t g_pga112_gain_code = PGA112_DEFAULT_GAIN_CODE;
static uint8_t g_pga112_channel_code = PGA112_ULTRASONIC_CHANNEL;

static uint16_t PGA112_BuildWriteWord(uint8_t gain_code, uint8_t channel_code);

/**
 * @brief 初始化PGA112，设置默认增益和通道
 * @note 该函数会先进入低功耗模式再恢复正常工作，以确保PGA112正确重置
 */
/**
 * @brief 初始化PGA112，设置默认增益和通道
 * @note 该函数会先进入低功耗模式再恢复正常工作，以确保PGA112正确重置
 */
void PGA112_Init(void)
{
    delay_ms(2);
    // 补充：先进入休眠状态
    PGA112_Shutdown(ENABLE);
    delay_ms(1); // 给予一定的稳定时间
    
    // 然后唤醒恢复正常工作
    PGA112_Shutdown(DISABLE);
    delay_ms(1); // 唤醒后也需要短暂稳定
    
    PGA112_WriteConfig(g_pga112_gain_code, g_pga112_channel_code);
}

/**
 * @brief 控制PGA112的电源状态
 * @param new_state ENABLE=进入低功耗模式，DISABLE=正常工作
 */
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

/**
 * @brief 写入PGA112配置寄存器，设置增益和输入通道
 * @param gain_code 增益代码(0-7)，对应增益1x到128x
 * @param channel_code 通道代码(0-15)，对应不同的输入通道
 */
void PGA112_WriteConfig(uint8_t gain_code, uint8_t channel_code)
{
    gain_code &= 0x07U;
    channel_code &= 0x0FU;

    g_pga112_gain_code = gain_code;
    g_pga112_channel_code = channel_code;

    User_SPI_Write16(PGA112_BuildWriteWord(gain_code, channel_code));
    delay_us(10);
}

/**
 * @brief 设置PGA增益代码
 * @param gain_code 增益代码(0-7)，对应增益1x到128x
 */
void PGA112_SetGainCode(uint8_t gain_code)
{
    PGA112_WriteConfig(gain_code, g_pga112_channel_code);
}

/**
 * @brief 设置PGA输入通道
 * @param channel_code 通道代码(0-15)，对应不同的输入通道
 */
void PGA112_SelectChannel(uint8_t channel_code)
{
    PGA112_WriteConfig(g_pga112_gain_code, channel_code);
}

/**
 * @brief 获取当前设置的PGA增益代码
 * @return 当前增益代码(0-7)，对应增益1x到128x
 */
uint8_t PGA112_GetGainCode(void)
{
    return g_pga112_gain_code;
}

/**
 * @brief 获取当前设置的PGA输入通道代码
 * @return 当前通道代码(0-15)，对应不同的输入通道
 */
uint8_t PGA112_GetChannelCode(void)
{
    return g_pga112_channel_code;
}

/**
 * @brief 根据增益代码获取对应的增益倍数
 * @param gain_code 增益代码(0-7)，对应增益1x到128x
 * @return 增益倍数，例如gain_code=0返回1，gain_code=7返回128
 */
uint16_t PGA112_GetGainValue(uint8_t gain_code)
{
    return (uint16_t)(1U << (gain_code & 0x07U));
}

/**
 * @brief 构建PGA112写命令字，包含增益和通道信息
 * @param gain_code 增益代码(0-7)，对应增益1x到128x
 * @param channel_code 通道代码(0-15)，对应不同的输入通道
 * @return 16位命令字，格式为[CMD_WRITE|GAIN_CODE|CHANNEL_CODE]
 */
static uint16_t PGA112_BuildWriteWord(uint8_t gain_code, uint8_t channel_code)
{
    return (uint16_t)(PGA112_CMD_WRITE | ((uint16_t)(gain_code & 0x07U) << 4) |
                      (uint16_t)(channel_code & 0x0FU));
}
