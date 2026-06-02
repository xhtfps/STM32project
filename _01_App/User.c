#include "User.h"

#define ULTRASONIC_TIMEOUT_US        12000U
#define ULTRASONIC_MIN_VALID_US      20U
#define ULTRASONIC_FILTER_SAMPLES    5U
#define ULTRASONIC_FLASH_ADDR        0x080E0000U
#define ULTRASONIC_FLASH_MAGIC       0x55534F4EU
#define ULTRASONIC_FLASH_VERSION     0x00010001U

// 屏幕显示常量定义
#define TITLE_STR        "超声波测距仪"
#define MODEL_VER_STR    "型号：HC-SR04"
#define USER_VER_STR     "版本：V1.0"
#define MENU1_CHOICE1    "1. 实时测量"
#define MENU1_CHOICE2    "2. 距离校准"
#define MENU1_CHOICE3    "3. 系统状态"
#define MENU_CHOICE_NUM  3
#define UI_BLANK_TEXT_16 "                                                                "
#define UI_BLANK_TEXT_24 "                                                "
#define UI_BLANK_TEXT_32 "                                    "

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t point_us[4];
    uint32_t reserved[2];
} UltrasonicCalibData;

static const uint16_t k_calib_distance_mm[4] = {100, 600, 900, 1300};

static volatile uint8_t g_echo_captured = 0;
static volatile uint8_t g_measure_active = 0;
static volatile uint32_t g_echo_time_us = 0;

static UltrasonicCalibData g_calib = {0};
static uint8_t g_calib_valid = 0;
static uint8_t g_menu_sign = 0;

static void Init_All(void);
static void Disp_Main(void);
static void Change_Menu(uint8_t menu_sign);
static void Clear_Work_Area(void);
static void Clear_Work_Text(void);
static void Draw_Work_Title(char *title);
static void Draw_Key_Tips(char *tip1, char *tip2);
static void Show_Text_Line(uint16_t line, char *text);
static void Show_Value_Line(uint16_t line, char *label, double value, char *format);

static void Ultrasonic_Timer_Init(void);
static void Ultrasonic_Echo_Init(void);
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us);
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us);
static void Sort_Samples(uint32_t *data, uint8_t length);

static void Calibration_Load(void);
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib);
static uint8_t Calibration_Save(const UltrasonicCalibData *calib);
static float Convert_Time_To_Distance(uint32_t echo_us);
static float Convert_Time_To_Distance_Default(uint32_t echo_us);

static void MenuHandler_Measure(void);
static void MenuHandler_Calibrate(void);
static void MenuHandler_Status(void);

void User_main(void)
{
    Init_All();
    Disp_Main();

    while(1)
    {
        switch(g_menu_sign)
        {
            case 0:
                if(Ps2KeyValue >= KeyValue_1 && Ps2KeyValue <= KeyValue_3)
                {
                    Change_Menu((uint8_t)(Ps2KeyValue - KeyValue_0));
                }
                break;
            case 1:
                MenuHandler_Measure();
                break;
            case 2:
                MenuHandler_Calibrate();
                break;
            case 3:
                MenuHandler_Status();
                break;
            default:
                g_menu_sign = 0;
                break;
        }
        delay_ms(10);
    }
}

static void Init_All(void)
{
    LCD_Clear(Black);
    Ultrasonic_PWM_Init();
    Ultrasonic_Timer_Init();
    Ultrasonic_Echo_Init();
    Calibration_Load();
}

static void Disp_Main(void)
{
    uint8_t count;
    OS_String_Show(272, 16, 32, 1, TITLE_STR);
    LCD_Appoint_Clear(0, 64, 800, 72, White);
    LCD_Appoint_Clear(0, 440, 800, 448, White);
    LCD_Appoint_Clear(250, 72, 252, 440, White);

    OS_String_Show(32, 456, 16, 1, MODEL_VER_STR);
    OS_String_Show(680, 456, 16, 1, USER_VER_STR);

    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    OS_String_Show(60, 96, 32, 1, MENU1_CHOICE1);
    OS_String_Show(60, 160, 32, 1, MENU1_CHOICE2);
    OS_String_Show(60, 224, 32, 1, MENU1_CHOICE3);
}

static void Change_Menu(uint8_t menu_sign)
{
    uint8_t count;

    Clear_Work_Area();
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    if(menu_sign >= 1 && menu_sign <= MENU_CHOICE_NUM)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * menu_sign), 32, 1, ">");
        g_menu_sign = menu_sign;
    }
    else
    {
        g_menu_sign = 0;
    }

    Ps2KeyValue = KeyValue_Null;
}

static void Clear_Work_Area(void)
{
    Clear_Work_Text();
}

static void Clear_Work_Text(void)
{
    uint8_t line;

    OS_String_Show(280, 88, 32, 1, UI_BLANK_TEXT_32);
    for(line = 0; line < 9U; line++)
    {
        OS_String_Show(280, (uint16_t)(150 + line * 30), 24, 1, UI_BLANK_TEXT_24);
    }
    OS_String_Show(280, 400, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 420, 16, 1, UI_BLANK_TEXT_16);
}

static void Draw_Work_Title(char *title)
{
    Clear_Work_Area();
    OS_String_Show(280, 88, 32, 1, title);
}

static void Draw_Key_Tips(char *tip1, char *tip2)
{
    OS_String_Show(280, 400, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 420, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 400, 16, 1, tip1);
    OS_String_Show(280, 420, 16, 1, tip2);
}

static void Show_Text_Line(uint16_t line, char *text)
{
    uint16_t y = (uint16_t)(150 + line * 30);
    OS_String_Show(280, y, 24, 1, UI_BLANK_TEXT_24);
    OS_String_Show(280, y, 24, 1, text);
}

static void Show_Value_Line(uint16_t line, char *label, double value, char *format)
{
    uint16_t y = (uint16_t)(150 + line * 30);
    OS_String_Show(280, y, 24, 1, UI_BLANK_TEXT_24);
    OS_String_Show(280, y, 24, 1, label);
    OS_Num_Show(500, y, 24, 1, value, format);
}

static void Ultrasonic_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = 84 - 1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base.TIM_Period = 0xFFFFFFFFU;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &tim_base);
    TIM_Cmd(TIM5, ENABLE);
}

static void Ultrasonic_Echo_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio_init);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

    EXTI_StructInit(&exti_init);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
    EXTI_ClearITPendingBit(EXTI_Line0);

    nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
    nvic_init.NVIC_IRQChannelSubPriority = 1;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}


static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us)
{
    uint32_t timeout;

    g_echo_captured = 0;
    g_measure_active = 1;
    g_echo_time_us = 0;

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

static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us)
{
    uint32_t samples[ULTRASONIC_FILTER_SAMPLES];
    uint8_t valid_count = 0;
    uint8_t attempts = 0;
    uint32_t sum = 0;
    uint8_t index;

    while(attempts < (ULTRASONIC_FILTER_SAMPLES + 3U) && valid_count < ULTRASONIC_FILTER_SAMPLES)
    {
        uint32_t sample = 0;
        attempts++;
        if(Ultrasonic_MeasureOnce(&sample) != 0U)
        {
            samples[valid_count++] = sample;
        }
        delay_ms(20);
    }

    if(valid_count == 0U)
    {
        return 0;
    }

    Sort_Samples(samples, valid_count);
    if(valid_count >= 3U)
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

    return 1;
}

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
                uint32_t temp = data[i];
                data[i] = data[j];
                data[j] = temp;
            }
        }
    }
}

static void Calibration_Load(void)
{
    const UltrasonicCalibData *stored = (const UltrasonicCalibData *)ULTRASONIC_FLASH_ADDR;
    g_calib = *stored;
    g_calib_valid = Calibration_IsValid(&g_calib);
}

static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib)
{
    uint8_t index;

    if(calib->magic != ULTRASONIC_FLASH_MAGIC || calib->version != ULTRASONIC_FLASH_VERSION)
    {
        return 0;
    }

    for(index = 0; index < 4U; index++)
    {
        if(calib->point_us[index] == 0U || calib->point_us[index] > ULTRASONIC_TIMEOUT_US)
        {
            return 0;
        }
        if(index > 0U && calib->point_us[index] <= calib->point_us[index - 1U])
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t Calibration_Save(const UltrasonicCalibData *calib)
{
    FLASH_Status status = FLASH_COMPLETE;
    const uint32_t *words = (const uint32_t *)calib;
    uint32_t address = ULTRASONIC_FLASH_ADDR;
    uint32_t index;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    if(status == FLASH_COMPLETE)
    {
        for(index = 0; index < (sizeof(UltrasonicCalibData) / 4U); index++)
        {
            status = FLASH_ProgramWord(address, words[index]);
            if(status != FLASH_COMPLETE)
            {
                break;
            }
            address += 4U;
        }
    }
    FLASH_Lock();

    return (uint8_t)(status == FLASH_COMPLETE);
}

static float Convert_Time_To_Distance_Default(uint32_t echo_us)
{
    float distance = (float)echo_us * 0.1715f;

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

static float Convert_Time_To_Distance(uint32_t echo_us)
{
    uint8_t index;
    float x0;
    float x1;
    float y0;
    float y1;
    float distance;

    if(g_calib_valid == 0U)
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    if(echo_us <= g_calib.point_us[0])
    {
        index = 0;
    }
    else if(echo_us <= g_calib.point_us[1])
    {
        index = 0;
    }
    else if(echo_us <= g_calib.point_us[2])
    {
        index = 1;
    }
    else
    {
        index = 2;
    }

    x0 = (float)g_calib.point_us[index];
    x1 = (float)g_calib.point_us[index + 1U];
    y0 = (float)k_calib_distance_mm[index];
    y1 = (float)k_calib_distance_mm[index + 1U];

    if(x1 <= x0)
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    distance = y0 + ((float)echo_us - x0) * (y1 - y0) / (x1 - x0);
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

// 测量模式
static void MenuHandler_Measure(void)
{
    char status_text[64];

    Draw_Work_Title("测量模式");
    Draw_Key_Tips("确认开始测量", "返回退出测量");
    Show_Text_Line(0, "测量准备...");

    while(Ps2KeyValue != KeyValue_Back)
    {
        uint32_t echo_us = 0;
        if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
        {
            float distance = Convert_Time_To_Distance(echo_us);
            Show_Value_Line(0, "测量时间(us)", echo_us, "%0.0f");
            Show_Value_Line(1, "测量距离(mm)", distance, "%0.1f");
            Show_Value_Line(2, "默认距离(mm)", Convert_Time_To_Distance_Default(echo_us), "%0.1f");
            sprintf(status_text, "校准状态: %s", (g_calib_valid != 0U) ? "已校准" : "未校准");
            Show_Text_Line(3, status_text);
        }
        else
        {
            Show_Text_Line(0, "测量失败");
            Show_Text_Line(1, "请检查超声波探头");
        }
        delay_ms(120);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

// 校准模式
static void MenuHandler_Calibrate(void)
{
    UltrasonicCalibData new_calib;
    uint8_t step = 0;
    char line[96];

    new_calib.magic = ULTRASONIC_FLASH_MAGIC;
    new_calib.version = ULTRASONIC_FLASH_VERSION;
    new_calib.point_us[0] = 0;
    new_calib.point_us[1] = 0;
    new_calib.point_us[2] = 0;
    new_calib.point_us[3] = 0;
    new_calib.reserved[0] = 0;
    new_calib.reserved[1] = 0;

    Draw_Work_Title("校准模式");
    Draw_Key_Tips("确认开始校准", "返回退出校准");

    while(Ps2KeyValue != KeyValue_Back)
    {
        sprintf(line, "请将探头对准%umm开始校准", k_calib_distance_mm[step]);
        Show_Text_Line(0, line);

        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;

            Show_Text_Line(1, "正在校准...");
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                new_calib.point_us[step] = echo_us;
                sprintf(line, "校准值：%0.0f us", (double)echo_us);
                Show_Text_Line(1, line);
                Show_Value_Line((uint16_t)(2 + step), "校准点时间(us)", echo_us, "%0.0f");

                step++;
                if(step >= 4U)
                {
                    if(Calibration_IsValid(&new_calib) != 0U && Calibration_Save(&new_calib) != 0U)
                    {
                        g_calib = new_calib;
                        g_calib_valid = 1;
                        Show_Text_Line(7, "校准完成！");
                    }
                    else
                    {
                        Show_Text_Line(7, "校准失败！");
                    }
                    delay_ms(1000);
                    break;
                }
            }
            else
            {
                Show_Text_Line(1, "校准失败，测量超时");
            }
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

// 状态模式
static void MenuHandler_Status(void)
{
    Draw_Work_Title("系统状态");
    Draw_Key_Tips("确认查看测量", "返回退出查看");

    Show_Text_Line(0, (g_calib_valid != 0U) ? "校准有效" : "未校准/校准数据无效");
    Show_Value_Line(1, "校准点1(us)", g_calib.point_us[0], "%0.0f");
    Show_Value_Line(2, "校准点2(us)", g_calib.point_us[1], "%0.0f");
    Show_Value_Line(3, "校准点3(us)", g_calib.point_us[2], "%0.0f");
    Show_Value_Line(4, "校准点4(us)", g_calib.point_us[3], "%0.0f");

    while(Ps2KeyValue != KeyValue_Back)
    {
        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                Show_Value_Line(5, "实时测量时间(us)", echo_us, "%0.0f");
                Show_Value_Line(6, "实时测量距离(mm)", Convert_Time_To_Distance(echo_us), "%0.1f");
            }
            else
            {
                Show_Text_Line(5, "测量失败");
            }
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

// 外部中断服务函数
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        if(g_measure_active != 0U)
        {
            uint32_t now = TIM_GetCounter(TIM5);
            if(now >= ULTRASONIC_MIN_VALID_US)
            {
                g_echo_time_us = now;
                g_echo_captured = 1;
                g_measure_active = 0;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
