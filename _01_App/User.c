#include "User.h"

/************************* 超声波模块参数配置 *************************/
// 测量超时时间(微秒)：对应最大测量距离约2米(声速343m/s，往返时间=2*2/343≈11662us)
#define ULTRASONIC_TIMEOUT_US        12000U
// 最小有效回波时间(微秒)：过滤近距离干扰和硬件噪声
#define ULTRASONIC_MIN_VALID_US      20U
#define ULTRASONIC_BLANKING_US       220U   // 发射+恢复后的有效接收起始窗
#define ULTRASONIC_PEAK_RATIO_NUM    1U     // 峰值位于回波脉冲包络中的经验位置k
#define ULTRASONIC_PEAK_RATIO_DEN    2U
// 滤波采样次数：采用多次采样+去极值平均，提高测量稳定性
#define ULTRASONIC_FILTER_SAMPLES    9U
#define ULTRASONIC_GAIN_RETRY_MAX    3U
// Flash存储地址：STM32F407的Sector11(0x080E0000-0x080FFFFF)，用于保存校准数据
#define ULTRASONIC_FLASH_ADDR        0x080E0000U
// Flash数据魔数：用于校验校准数据的有效性
#define ULTRASONIC_FLASH_MAGIC       0x55534F4EU  // "USON"的ASCII码
// Flash数据版本号：用于兼容不同版本的校准数据格式
#define ULTRASONIC_FLASH_VERSION     0x00010001U  // 主版本1.0，次版本1

/************************* 屏幕显示常量定义 *************************/
#define TITLE_STR        "超声波测距仪"       // 主界面标题
#define MODEL_VER_STR    "型号：HC-SR04"      // 硬件型号
#define USER_VER_STR     "版本：V1.0"         // 软件版本
#define MENU1_CHOICE1    "1. 实时测量"        // 菜单选项1
#define MENU1_CHOICE2    "2. 距离校准"        // 菜单选项2
#define MENU1_CHOICE3    "3. 系统状态"        // 菜单选项3
#define MENU_CHOICE_NUM  3                    // 菜单选项总数
// 不同字号的空白字符串，用于快速清屏指定区域
#define UI_BLANK_TEXT_16 "                                                                "  // 16号字，64个字符
#define UI_BLANK_TEXT_24 "                                                "                // 24号字，48个字符
#define UI_BLANK_TEXT_32 "                                    "                            // 32号字，32个字符

/************************* 数据结构定义 *************************/
/**
 * @brief 超声波校准数据结构体
 * @note 存储在Flash中，用于分段线性校准距离测量值
 */
typedef struct
{
    uint32_t magic;               // 数据魔数，用于校验
    uint32_t version;             // 数据版本号
    uint32_t point_us[4];         // 4个校准点对应的回波时间(微秒)
    uint32_t reserved[2];         // 保留字段，用于未来扩展
} UltrasonicCalibData;

// 4个校准点对应的标准距离(毫米)
static const uint16_t k_calib_distance_mm[4] = {100, 600, 900, 1300};

/************************* 全局变量定义 *************************/
// 超声波测量状态变量
static volatile uint8_t g_echo_captured = 0;    // 回波捕获标志：1=已捕获有效回波
static volatile uint8_t g_measure_active = 0;   // 测量进行中标志：1=正在测量
static volatile uint32_t g_echo_time_us = 0;    // 估算得到的回波到达时间(微秒)
static volatile uint32_t g_echo_rise_us = 0;    // 回波脉冲首边沿时间(微秒)
static volatile uint32_t g_echo_fall_us = 0;    // 回波脉冲终边沿时间(微秒)
static volatile uint8_t g_echo_rise_seen = 0;   // 回波首边沿已到达标志
static uint8_t g_gain_settle_discard = 0;       // PGA增益切换后需丢弃一次测量
static uint32_t g_last_echo_us = 1500U;         // 上一次有效回波时间，用于预测下次增益
static uint8_t g_ultrasonic_gain_code = PGA112_DEFAULT_GAIN_CODE;

// 校准数据变量
static UltrasonicCalibData g_calib = {0};       // 当前生效的校准数据
static uint8_t g_calib_valid = 0;               // 校准数据有效标志：1=有效

// UI状态变量
static uint8_t g_menu_sign = 0;                 // 当前菜单索引：0=主菜单，1=测量，2=校准，3=状态

/************************* 函数声明 *************************/
// 系统初始化与主界面函数
static void Init_All(void);
static void Disp_Main(void);
static void Change_Menu(uint8_t menu_sign);

// UI工具函数
static void Clear_Work_Area(void);
static void Clear_Work_Text(void);
static void Draw_Work_Title(char *title);
static void Draw_Key_Tips(char *tip1, char *tip2);
static void Show_Text_Line(uint16_t line, char *text);
static void Show_Value_Line(uint16_t line, char *label, double value, char *format);

// 超声波硬件驱动函数
static void Ultrasonic_Timer_Init(void);
static void Ultrasonic_Echo_Init(void);
static void Ultrasonic_ApplyGain(uint8_t gain_code);
static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us);
static uint16_t Ultrasonic_GetGainValue(uint8_t gain_code);
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us);
static void Ultrasonic_PrepareGain(uint8_t retry_count);
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us);
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us);
static void Sort_Samples(uint32_t *data, uint8_t length);

// 校准与距离转换函数
static void Calibration_Load(void);
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib);
static uint8_t Calibration_Save(const UltrasonicCalibData *calib);
static float Convert_Time_To_Distance(uint32_t echo_us);
static float Convert_Time_To_Distance_Default(uint32_t echo_us);

// 菜单处理函数
static void MenuHandler_Measure(void);
static void MenuHandler_Calibrate(void);
static void MenuHandler_Status(void);

/************************* 主函数 *************************/
/**
 * @brief 用户主函数，程序入口
 * @note 初始化所有模块后进入主循环，根据菜单状态调用对应处理函数
 */
void User_main(void)
{
    Init_All();          // 初始化所有硬件和软件模块
    Disp_Main();         // 显示主菜单界面

    while(1)
    {
        switch(g_menu_sign)
        {
            case 0:  // 主菜单状态：处理按键选择
                if(Ps2KeyValue >= KeyValue_1 && Ps2KeyValue <= KeyValue_3)
                {
                    // 根据按键值切换到对应菜单
                    Change_Menu((uint8_t)(Ps2KeyValue - KeyValue_0));
                }
                break;
            case 1:  // 实时测量模式
                MenuHandler_Measure();
                break;
            case 2:  // 距离校准模式
                MenuHandler_Calibrate();
                break;
            case 3:  // 系统状态模式
                MenuHandler_Status();
                break;
            default: // 异常状态：返回主菜单
                g_menu_sign = 0;
                break;
        }
        delay_ms(10);  // 主循环延时，防止CPU占用过高
    }
}

/************************* 系统初始化函数 *************************/
/**
 * @brief 初始化所有硬件和软件模块
 */
static void Init_All(void)
{
    LCD_Clear(Black);                  // 清屏，背景黑色
    Ultrasonic_PWM_Init();             // 初始化超声波触发引脚(PWM输出)
    Ultrasonic_Timer_Init();           // 初始化定时器5，用于计时回波时间
    Ultrasonic_Echo_Init();            // 初始化回波引脚和外部中断
    Calibration_Load();                // 从Flash加载校准数据
    Ultrasonic_ApplyGain(PGA112_DEFAULT_GAIN_CODE);
    g_gain_settle_discard = 0;
}

/************************* 主界面显示函数 *************************/
/**
 * @brief 绘制主菜单界面
 */
static void Disp_Main(void)
{
    uint8_t count;
    
    // 显示标题
    OS_String_Show(272, 16, 32, 1, TITLE_STR);
    
    // 绘制界面分割线
    LCD_Appoint_Clear(0, 64, 800, 72, White);    // 顶部横线
    LCD_Appoint_Clear(0, 440, 800, 448, White);  // 底部横线
    LCD_Appoint_Clear(250, 72, 252, 440, White); // 左侧菜单与右侧工作区分隔线
    
    // 显示底部信息
    OS_String_Show(32, 456, 16, 1, MODEL_VER_STR);  // 左下角显示型号
    OS_String_Show(680, 456, 16, 1, USER_VER_STR);  // 右下角显示版本
    
    // 绘制菜单选择标记(初始为"-")
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }
    
    // 显示菜单选项
    OS_String_Show(60, 96, 32, 1, MENU1_CHOICE1);
    OS_String_Show(60, 160, 32, 1, MENU1_CHOICE2);
    OS_String_Show(60, 224, 32, 1, MENU1_CHOICE3);
}

/**
 * @brief 切换菜单并更新界面
 * @param menu_sign 目标菜单索引
 */
static void Change_Menu(uint8_t menu_sign)
{
    uint8_t count;

    Clear_Work_Area();  // 清空右侧工作区
    
    // 重置所有菜单选择标记为"-"
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    // 如果是有效菜单索引，更新选择标记为">"并切换菜单
    if(menu_sign >= 1 && menu_sign <= MENU_CHOICE_NUM)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * menu_sign), 32, 1, ">");
        g_menu_sign = menu_sign;
    }
    else
    {
        g_menu_sign = 0;  // 无效索引，返回主菜单
    }

    Ps2KeyValue = KeyValue_Null;  // 清除按键状态
}

/************************* UI工具函数 *************************/
/**
 * @brief 清空右侧工作区
 */
static void Clear_Work_Area(void)
{
    Clear_Work_Text();  // 清空工作区所有文本
}

/**
 * @brief 清空工作区所有文本内容
 */
static void Clear_Work_Text(void)
{
    uint8_t line;

    // 清空标题行(32号字)
    OS_String_Show(280, 88, 32, 1, UI_BLANK_TEXT_32);
    
    // 清空9行内容行(24号字)
    for(line = 0; line < 9U; line++)
    {
        OS_String_Show(280, (uint16_t)(150 + line * 30), 24, 1, UI_BLANK_TEXT_24);
    }
    
    // 清空底部两行按键提示(16号字)
    OS_String_Show(280, 400, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 420, 16, 1, UI_BLANK_TEXT_16);
}

/**
 * @brief 绘制工作区标题
 * @param title 标题字符串
 */
static void Draw_Work_Title(char *title)
{
    Clear_Work_Area();  // 先清空工作区
    OS_String_Show(280, 88, 32, 1, title);  // 显示标题
}

/**
 * @brief 绘制底部按键提示
 * @param tip1 第一行提示
 * @param tip2 第二行提示
 */
static void Draw_Key_Tips(char *tip1, char *tip2)
{
    // 先清空提示区域
    OS_String_Show(280, 400, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 420, 16, 1, UI_BLANK_TEXT_16);
    
    // 显示新的提示
    OS_String_Show(280, 400, 16, 1, tip1);
    OS_String_Show(280, 420, 16, 1, tip2);
}

/**
 * @brief 在工作区指定行显示文本
 * @param line 行号(0-8)
 * @param text 要显示的文本
 */
static void Show_Text_Line(uint16_t line, char *text)
{
    uint16_t y = (uint16_t)(150 + line * 30);  // 计算Y坐标
    OS_String_Show(280, y, 24, 1, UI_BLANK_TEXT_24);  // 先清空该行
    OS_String_Show(280, y, 24, 1, text);  // 显示文本
}

/**
 * @brief 在工作区指定行显示带标签的数值
 * @param line 行号(0-8)
 * @param label 标签文本
 * @param value 要显示的数值
 * @param format 数值格式化字符串
 */
static void Show_Value_Line(uint16_t line, char *label, double value, char *format)
{
    uint16_t y = (uint16_t)(150 + line * 30);  // 计算Y坐标
    OS_String_Show(280, y, 24, 1, UI_BLANK_TEXT_24);  // 先清空该行
    OS_String_Show(280, y, 24, 1, label);  // 显示标签
    OS_Num_Show(500, y, 24, 1, value, format);  // 显示数值
}

/************************* 超声波硬件驱动函数 *************************/
/**
 * @brief 初始化定时器5，用于精确计时回波时间
 * @note 定时器时钟为84MHz，预分频84，计数频率为1MHz，即1us计数一次
 */
static void Ultrasonic_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  // 使能TIM5时钟

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 84 - 1;                     // 预分频器：84分频
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;       // 向上计数模式
    tim_base.TIM_Period = 0xFFFFFFFFU;                   // 自动重装值：最大32位
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;           // 时钟不分频
    TIM_TimeBaseInit(TIM5, &tim_base);
    
    TIM_Cmd(TIM5, ENABLE);  // 启动定时器
}

/**
 * @brief 初始化超声波回波引脚和外部中断
 * @note 回波引脚：PC0，双边沿触发中断，用于提取脉冲首尾边沿
 */
static void Ultrasonic_Echo_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    // 使能GPIO和SYSCFG时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // 配置PC0为输入模式，无上拉下拉
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio_init);

    // 配置EXTI线0连接到PC0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

    // 配置EXTI线0为双边沿触发中断
    EXTI_StructInit(&exti_init);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // 记录脉冲起止边沿
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
    EXTI_ClearITPendingBit(EXTI_Line0);  // 清除中断标志

    // 配置NVIC中断优先级
    nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2;  // 抢占优先级2
    nvic_init.NVIC_IRQChannelSubPriority = 1;         // 子优先级1
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

/**
 * @brief 执行一次超声波测量
 * @param echo_us 输出参数，返回测量到的回波时间(微秒)
 * @return 0=测量超时，1=测量成功
 */
static void Ultrasonic_ApplyGain(uint8_t gain_code)
{
    gain_code &= 0x07U;
    if(gain_code != g_ultrasonic_gain_code)
    {
        PGA112_SetGainCode(gain_code);
        g_ultrasonic_gain_code = gain_code;
        g_gain_settle_discard = 1;
    }
}

static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us)
{
    if(echo_us < 180U)
    {
        return PGA112_GAIN_1;
    }
    if(echo_us < 360U)
    {
        return PGA112_GAIN_2;
    }
    if(echo_us < 800U)
    {
        return PGA112_GAIN_4;
    }
    if(echo_us < 1800U)
    {
        return PGA112_GAIN_8;
    }
    if(echo_us < 3600U)
    {
        return PGA112_GAIN_16;
    }
    if(echo_us < 6500U)
    {
        return PGA112_GAIN_32;
    }
    if(echo_us < 9500U)
    {
        return PGA112_GAIN_64;
    }
    return PGA112_GAIN_128;
}

static uint16_t Ultrasonic_GetGainValue(uint8_t gain_code)
{
    return PGA112_GetGainValue(gain_code);
}
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us)
{
    uint32_t width;

    if(fall_us <= rise_us)
    {
        return rise_us;
    }

    width = fall_us - rise_us;
    return rise_us + (width * ULTRASONIC_PEAK_RATIO_NUM) / ULTRASONIC_PEAK_RATIO_DEN;
}

static void Ultrasonic_PrepareGain(uint8_t retry_count)
{
    uint8_t gain_code = Ultrasonic_SelectGainCode(g_last_echo_us);

    if(retry_count > ULTRASONIC_GAIN_RETRY_MAX)
    {
        retry_count = ULTRASONIC_GAIN_RETRY_MAX;
    }

    gain_code = (uint8_t)(gain_code + retry_count);
    if(gain_code > PGA112_GAIN_128)
    {
        gain_code = PGA112_GAIN_128;
    }

    Ultrasonic_ApplyGain(gain_code);
    delay_us(20);
}

static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us)
{
    uint32_t timeout;

    g_echo_captured = 0;
    g_measure_active = 1;
    g_echo_time_us = 0;
    g_echo_rise_us = 0;
    g_echo_fall_us = 0;
    g_echo_rise_seen = 0;

    TIM_SetCounter(TIM5, 0);
    EXTI_ClearITPendingBit(EXTI_Line0);
    Ultrasonic_FireBurst();

    for(timeout = 0; timeout < ULTRASONIC_TIMEOUT_US / 10U; timeout++)
    {
        if(g_echo_captured != 0U)
        {
            *echo_us = g_echo_time_us;
            g_measure_active = 0;
            return 1;
        }
        delay_us(10);
    }

    g_measure_active = 0;
    return 0;
}

/**
 * @brief 执行带滤波的超声波测量
 * @param echo_us 输出参数，返回滤波后的回波时间(微秒)
 * @return 0=所有采样都失败，1=测量成功
 * @note 采用"多次采样+排序+去极值平均"的滤波算法，提高测量稳定性
 */
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us)
{
    uint32_t samples[ULTRASONIC_FILTER_SAMPLES];
    uint8_t valid_count = 0;
    uint8_t attempts = 0;
    uint8_t gain_retry = 0;
    uint32_t sum = 0;
    uint8_t index;

    while(attempts < (ULTRASONIC_FILTER_SAMPLES + 6U) && valid_count < ULTRASONIC_FILTER_SAMPLES)
    {
        uint32_t sample = 0;

        Ultrasonic_PrepareGain(gain_retry);
        attempts++;

        if(Ultrasonic_MeasureOnce(&sample) != 0U)
        {
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
                delay_ms(20);
                continue;
            }

            samples[valid_count++] = sample;
            g_last_echo_us = sample;
            gain_retry = 0;
        }
        else
        {
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
            }
            else if(gain_retry < ULTRASONIC_GAIN_RETRY_MAX)
            {
                gain_retry++;
            }
        }
        delay_ms(20);
    }

    if(valid_count == 0U)
    {
        return 0;
    }

    Sort_Samples(samples, valid_count);
    if(valid_count >= 5U)
    {
        for(index = 2; index < (uint8_t)(valid_count - 2U); index++)
        {
            sum += samples[index];
        }
        *echo_us = sum / (valid_count - 4U);
    }
    else if(valid_count >= 3U)
    {
        for(index = 1; index < (uint8_t)(valid_count - 1U); index++)
        {
            sum += samples[index];
        }
        *echo_us = sum / (valid_count - 2U);
    }
    else
    {
        *echo_us = samples[valid_count / 2U];
    }

    g_last_echo_us = *echo_us;
    return 1;
}

/**
 * @brief 对采样数据进行升序排序(冒泡排序)
 * @param data 待排序的数据数组
 * @param length 数组长度
 */
static void Sort_Samples(uint32_t *data, uint8_t length)
{
    uint8_t i;
    uint8_t j;

    for(i = 0; i < length; i++)
    {
        for(j = (uint8_t)(i + 1U); j < length; j++)
        {
            if(data[j] < data[i])
            {
                // 交换两个元素
                uint32_t temp = data[i];
                data[i] = data[j];
                data[j] = temp;
            }
        }
    }
}

/************************* 校准与距离转换函数 *************************/
/**
 * @brief 从Flash加载校准数据
 */
static void Calibration_Load(void)
{
    // 将Flash地址强制转换为结构体指针，直接读取数据
    const UltrasonicCalibData *stored = (const UltrasonicCalibData *)ULTRASONIC_FLASH_ADDR;
    g_calib = *stored;  // 复制到全局变量
    g_calib_valid = Calibration_IsValid(&g_calib);  // 校验数据有效性
}

/**
 * @brief 校验校准数据的有效性
 * @param calib 待校验的校准数据
 * @return 0=无效，1=有效
 */
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib)
{
    uint8_t index;

    // 校验魔数和版本号
    if(calib->magic != ULTRASONIC_FLASH_MAGIC || calib->version != ULTRASONIC_FLASH_VERSION)
    {
        return 0;
    }

    // 校验每个校准点的时间值
    for(index = 0; index < 4U; index++)
    {
        // 时间值不能为0，也不能超过最大超时时间
        if(calib->point_us[index] == 0U || calib->point_us[index] > ULTRASONIC_TIMEOUT_US)
        {
            return 0;
        }
        // 时间值必须严格递增(距离越远，回波时间越长)
        if(index > 0U && calib->point_us[index] <= calib->point_us[index - 1U])
        {
            return 0;
        }
    }

    return 1;  // 所有校验通过
}

/**
 * @brief 将校准数据保存到Flash
 * @param calib 要保存的校准数据
 * @return 0=保存失败，1=保存成功
 */
static uint8_t Calibration_Save(const UltrasonicCalibData *calib)
{
    FLASH_Status status = FLASH_COMPLETE;
    const uint32_t *words = (const uint32_t *)calib;  // 按32位字访问
    uint32_t address = ULTRASONIC_FLASH_ADDR;
    uint32_t index;

    FLASH_Unlock();  // 解锁Flash
    
    // 清除所有Flash操作标志
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    
    // 擦除Sector11(必须先擦除才能写入)
    status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    
    if(status == FLASH_COMPLETE)
    {
        // 按字写入校准数据
        for(index = 0; index < (sizeof(UltrasonicCalibData) / 4U); index++)
        {
            status = FLASH_ProgramWord(address, words[index]);
            if(status != FLASH_COMPLETE)
            {
                break;  // 写入失败，退出
            }
            address += 4U;  // 下一个字地址
        }
    }
    
    FLASH_Lock();  // 锁定Flash

    return (uint8_t)(status == FLASH_COMPLETE);  // 返回操作结果
}

/**
 * @brief 使用默认公式将回波时间转换为距离
 * @param echo_us 回波时间(微秒)
 * @return 距离(毫米)
 * @note 默认公式：距离(mm) = 时间(us) * 0.1715
 *       声速343m/s = 0.343mm/us，往返时间所以除以2，即0.343/2=0.1715
 */
static float Convert_Time_To_Distance_Default(uint32_t echo_us)
{
    float distance = (float)echo_us * 0.1715f;

    // 限制距离范围：10mm ~ 1300mm
    if(distance < 10.0f)
    {
        distance = 10.0f;
    }
    if(distance > 1300.0f)
    {
        distance = 1300.0f;
    }
    return distance;
}

/**
 * @brief 使用校准数据将回波时间转换为距离
 * @param echo_us 回波时间(微秒)
 * @return 距离(毫米)
 * @note 采用分段线性插值算法，提高测量精度
 */
static float Convert_Time_To_Distance(uint32_t echo_us)
{
    uint8_t index;
    float x0, x1;  // 两个校准点的时间值
    float y0, y1;  // 两个校准点的距离值
    float distance;

    // 如果校准数据无效，使用默认公式
    if(g_calib_valid == 0U)
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    // 确定当前时间所在的校准区间
    if(echo_us <= g_calib.point_us[0])
    {
        index = 0;  // 小于第一个校准点，使用第一区间外推
    }
    else if(echo_us <= g_calib.point_us[1])
    {
        index = 0;  // 在第一区间内
    }
    else if(echo_us <= g_calib.point_us[2])
    {
        index = 1;  // 在第二区间内
    }
    else
    {
        index = 2;  // 在第三区间内或大于第三个校准点
    }

    // 获取区间两端的校准点
    x0 = (float)g_calib.point_us[index];
    x1 = (float)g_calib.point_us[index + 1U];
    y0 = (float)k_calib_distance_mm[index];
    y1 = (float)k_calib_distance_mm[index + 1U];

    // 防止除零错误
    if(x1 <= x0)
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    // 线性插值计算距离
    distance = y0 + ((float)echo_us - x0) * (y1 - y0) / (x1 - x0);
    
    // 限制距离范围：10mm ~ 1300mm
    if(distance < 10.0f)
    {
        distance = 10.0f;
    }
    if(distance > 1300.0f)
    {
        distance = 1300.0f;
    }
    return distance;
}

/************************* 菜单处理函数 *************************/
/**
 * @brief 实时测量模式处理函数
 */
static void MenuHandler_Measure(void)
{
    char status_text[64];

    // 绘制测量模式界面
    Draw_Work_Title("测量模式");
    Draw_Key_Tips("确认开始测量", "返回退出测量");
    Show_Text_Line(0, "测量准备...");

    // 循环测量，直到按下返回键
    while(Ps2KeyValue != KeyValue_Back)
    {
        uint32_t echo_us = 0;
        if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
        {
            // 测量成功，显示结果
            float distance = Convert_Time_To_Distance(echo_us);
            Show_Value_Line(0, "测量时间(us)", echo_us, "%0.0f");
            Show_Value_Line(1, "测量距离(mm)", distance, "%0.1f");
            Show_Value_Line(2, "默认距离(mm)", Convert_Time_To_Distance_Default(echo_us), "%0.1f");
            
            // 显示校准状态
            sprintf(status_text, "校准状态: %s", (g_calib_valid != 0U) ? "已校准" : "未校准");
            Show_Text_Line(3, status_text);
            sprintf(status_text, "前端增益: %ux", Ultrasonic_GetGainValue(g_ultrasonic_gain_code));
            Show_Text_Line(4, status_text);
        }
        else
        {
            // 测量失败，显示错误信息
            Show_Text_Line(0, "测量失败");
            Show_Text_Line(1, "请检查超声波探头");
            sprintf(status_text, "前端增益: %ux", Ultrasonic_GetGainValue(g_ultrasonic_gain_code));
            Show_Text_Line(2, status_text);
        }
        delay_ms(120);  // 测量间隔120ms
    }

    Ps2KeyValue = KeyValue_Null;  // 清除按键状态
    Change_Menu(0);               // 返回主菜单
}

/**
 * @brief 距离校准模式处理函数
 */
static void MenuHandler_Calibrate(void)
{
    UltrasonicCalibData new_calib;  // 新的校准数据
    uint8_t step = 0;               // 当前校准步骤(0-3)
    char line[96];

    // 初始化新的校准数据
    new_calib.magic = ULTRASONIC_FLASH_MAGIC;
    new_calib.version = ULTRASONIC_FLASH_VERSION;
    new_calib.point_us[0] = 0;
    new_calib.point_us[1] = 0;
    new_calib.point_us[2] = 0;
    new_calib.point_us[3] = 0;
    new_calib.reserved[0] = 0;
    new_calib.reserved[1] = 0;

    // 绘制校准模式界面
    Draw_Work_Title("校准模式");
    Draw_Key_Tips("确认开始校准", "返回退出校准");

    // 循环校准，直到按下返回键或校准完成
    while(Ps2KeyValue != KeyValue_Back)
    {
        // 显示当前校准步骤提示
        sprintf(line, "请将探头对准%umm开始校准", k_calib_distance_mm[step]);
        Show_Text_Line(0, line);

        // 按下确认键，执行当前点校准
        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;

            Show_Text_Line(1, "正在校准...");
            
            // 测量当前点的回波时间
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                // 保存校准值
                new_calib.point_us[step] = echo_us;
                sprintf(line, "校准值：%0.0f us", (double)echo_us);
                Show_Text_Line(1, line);
                Show_Value_Line((uint16_t)(2 + step), "校准点时间(us)", echo_us, "%0.0f");

                step++;  // 进入下一个校准点
                if(step >= 4U)
                {
                    // 所有校准点完成，校验并保存数据
                    if(Calibration_IsValid(&new_calib) != 0U && Calibration_Save(&new_calib) != 0U)
                    {
                        // 保存成功，更新全局校准数据
                        g_calib = new_calib;
                        g_calib_valid = 1;
                        Show_Text_Line(7, "校准完成！");
                    }
                    else
                    {
                        Show_Text_Line(7, "校准失败！");
                    }
                    delay_ms(1000);  // 显示结果1秒
                    break;  // 退出校准流程
                }
            }
            else
            {
                Show_Text_Line(1, "校准失败，测量超时");
            }
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;  // 清除按键状态
    Change_Menu(0);               // 返回主菜单
}

/**
 * @brief 系统状态模式处理函数
 */
static void MenuHandler_Status(void)
{
    // 绘制系统状态界面
    Draw_Work_Title("系统状态");
    Draw_Key_Tips("确认查看测量", "返回退出查看");

    // 显示校准状态和校准点数据
    Show_Text_Line(0, (g_calib_valid != 0U) ? "校准有效" : "未校准/校准数据无效");
    Show_Value_Line(1, "校准点1(us)", g_calib.point_us[0], "%0.0f");
    Show_Value_Line(2, "校准点2(us)", g_calib.point_us[1], "%0.0f");
    Show_Value_Line(3, "校准点3(us)", g_calib.point_us[2], "%0.0f");
    Show_Value_Line(4, "校准点4(us)", g_calib.point_us[3], "%0.0f");
    Show_Value_Line(5, "当前增益(x)", Ultrasonic_GetGainValue(g_ultrasonic_gain_code), "%0.0f");

    // 循环等待按键，直到按下返回键
    while(Ps2KeyValue != KeyValue_Back)
    {
        // 按下确认键，执行一次实时测量
        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                // 显示实时测量结果
                Show_Value_Line(6, "实时测量时间(us)", echo_us, "%0.0f");
                Show_Value_Line(7, "实时测量距离(mm)", Convert_Time_To_Distance(echo_us), "%0.1f");
            }
            else
            {
                Show_Text_Line(6, "测量失败");
            }
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;  // 清除按键状态
    Change_Menu(0);               // 返回主菜单
}

/************************* 中断服务函数 *************************/
/**
 * @brief EXTI0外部中断服务函数
 * @note 当回波引脚(PC0)出现边沿时触发：上升沿记录起点，下降沿记录终点
 */
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        if(g_measure_active != 0U)
        {
            uint32_t now = TIM_GetCounter(TIM5);

            if(now >= ULTRASONIC_BLANKING_US)
            {
                if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) != Bit_RESET)
                {
                    if(g_echo_rise_seen == 0U)
                    {
                        g_echo_rise_us = now;
                        g_echo_rise_seen = 1U;
                    }
                }
                else if(g_echo_rise_seen != 0U && now > g_echo_rise_us)
                {
                    g_echo_fall_us = now;
                    g_echo_time_us = Ultrasonic_EstimatePeakTime(g_echo_rise_us, g_echo_fall_us);
                    if(g_echo_time_us >= ULTRASONIC_MIN_VALID_US)
                    {
                        g_echo_captured = 1U;
                        g_measure_active = 0U;
                    }
                }
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
