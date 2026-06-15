#include "User.h"
#include "Drive_PWM.h"

/************************* 超声波模块参数配置 *************************/
#include "User.h"
#include "Drive_PWM.h"

/************************* 超声波模块参数配置 *************************/
/*
 * 物理原理：常温空气中声速 ≈ 343 m/s
 * 1us 声波单程距离：343 * 100 / 1000000 = 0.0343 cm = 0.343 mm
 * 超声波往返测距，因此 1us 对应实际距离：0.343 / 2 = 0.1715 mm
 */
// 测量超时时间(us)：对应最大测距约2m，理论往返时间11662us，预留余量设为12000us
#define ULTRASONIC_TIMEOUT_US        12000U
// 最小有效回波时间(us)：过滤电路抖动、杂波产生的极短干扰脉冲
#define ULTRASONIC_MIN_VALID_US      20U
// 回波脉冲最小宽度(us)：小于该值判定为毛刺干扰，丢弃
#define ULTRASONIC_MIN_PULSE_WIDTH_US  30U
// 发射盲区/消隐时间(us)：探头发射后存在机械余振+电路自激，此时间段屏蔽接收
// 450us 盲区对应近距离约 7.7cm，避免近距离自激误触发
#define ULTRASONIC_BLANKING_US       450U   
// 采样滤波点数：单次有效测距采集多组样本，配合极值、聚类算法降噪
#define ULTRASONIC_FILTER_SAMPLES    40U
// 样本聚类区间(us)：判断两组采样值是否属于同一个有效回波簇
#define ULTRASONIC_CLUSTER_SPAN_US   80U
#define ULTRASONIC_CLUSTER_MIN_COUNT 6U
// 增益最大重试次数：回波微弱未检测到时，逐级提升PGA增益的最大尝试次数
#define ULTRASONIC_GAIN_RETRY_MAX    3U
// 跟踪窗口余量(us)：锁定有效回波后，在历史值基础上扩大窗口范围，动态跟踪目标
#define ULTRASONIC_TRACK_MARGIN_US   4000U
// 连续丢失回波次数阈值：超过该值判定目标丢失，重新全域搜索
#define ULTRASONIC_REACQUIRE_MISSES  2U
// 重搜索模式下最小接收时间(us)：屏蔽近距离自激干扰，专用于远距离搜索
#define ULTRASONIC_REACQUIRE_MIN_US  650U
// 校准数据Flash存储地址：STM32F407 Sector11起始地址(Flash最后128KB扇区)
// 选用末尾扇区，避免与程序代码区(0x08000000开始)冲突
#define ULTRASONIC_FLASH_ADDR        0x080E0000U
// Flash数据魔术字：ASCII "USON" (0x55 0x53 0x4F 0x4E)
// 上电校验魔术字，判断Flash区域是否为合法校准数据，区分空/乱码
#define ULTRASONIC_FLASH_MAGIC       0x55534F4EU  
// 数据版本号：结构体/字段修改时升级版本，让旧版校准数据自动失效，防止解析错误
#define ULTRASONIC_FLASH_VERSION     0x00010004U  

/************************* 界面显示字符串定义 *************************/
#define TITLE_STR        "超声波测距仪"             //主界面顶部标题
#define MODEL_VER_STR    "型号：HC-SR04"            //硬件型号标注
#define USER_VER_STR     "版本：V1.0"               //软件版本号
#define MENU1_CHOICE1    "1. 手动测量"              //菜单选项1
#define MENU1_CHOICE2    "2. 距离校准"              //菜单选项2
#define MENU1_CHOICE3    "3. 实时测量"              //菜单选项3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
#define MENU1_CHOICE4    "4. 程控调节"              //菜单选项4
#define MENU_CHOICE_NUM  4                         //菜单总数量

// 局部清屏空白串：用空格覆盖原有字符，比全局清屏LCD_Clear效率更高，减少闪屏
#define UI_BLANK_TEXT_16 "                                                                "  // 16号字体 64字符空格
#define UI_BLANK_TEXT_24 "                                                "                // 24号字体 48字符空格
#define UI_BLANK_TEXT_32 "                                "                                // 32号字体 32字符空格
#define UI_VALUE_BLANK_24 "                        "                                       // 数值区域专用空白串

/************************* 数据结构定义 *************************/

/**
 * @brief 超声波分段线性校准数据结构体
 * @note 用于修正硬件延迟、声速偏差、电路非线性误差，采用多点分段插值校准
 * @attention 结构体自然4字节对齐，适配Flash 32bit写入规则
 */
typedef struct
{
    uint32_t magic;               // 数据合法性魔术标记
    uint32_t version;             // 结构体版本号
    uint32_t point_us[5];         // 存储5个标准距离下实际测得的回波时间(us)
    uint32_t reserved[1];         // 预留空间，保证结构体总大小32字节
} UltrasonicCalibData;

// 5组基准校准距离(单位：mm)，覆盖近、中、远全量程区间
static const uint16_t k_calib_distance_mm[5] = {100, 300, 600, 900, 1300};

/************************* 全局变量定义 *************************/
/*
 * volatile 关键字说明：
 * 以下变量均在【外部中断】中修改，volatile强制编译器每次从内存读取，
 * 禁止编译器优化，防止中断与主循环变量不同步、程序卡死。
 */
static volatile uint8_t g_echo_captured = 0;    // 回波采集完成标记 1=成功 0=未完成
static volatile uint8_t g_measure_active = 0;   // 测量窗口使能标记 1=允许中断接收回波 0=屏蔽
static volatile uint32_t g_echo_time_us = 0;    // 最终计算得到的回波峰值时间(us)
static volatile uint32_t g_echo_rise_us = 0;    // 回波脉冲上升沿时刻(us)
static volatile uint32_t g_echo_fall_us = 0;    // 回波脉冲下降沿时刻(us)
static volatile uint8_t g_echo_rise_seen = 0;   // 状态机标记：是否已检测到上升沿
static volatile uint32_t g_echo_accept_min_us = 0;  // 回波接收窗口下限(us)
static volatile uint32_t g_echo_accept_max_us = ULTRASONIC_TIMEOUT_US; // 回波接收窗口上限(us)

static uint8_t g_gain_settle_discard = 0;       // 增益切换标记 1=丢弃本次测量(运放电路需要稳定时间)
static uint32_t g_last_echo_us = 1500U;         // 上一次有效回波时间，用于预测当前增益档位
static uint8_t g_tracking_valid = 0;           // 跟踪窗口标记 1=已锁定有效回波，开启窄窗口跟踪
static uint8_t g_reacquire_ignore_near = 0;     // 重搜索标记 1=屏蔽近距离信号，专注远距离搜索
static uint8_t g_ultrasonic_gain_code = PGA112_DEFAULT_GAIN_CODE; // 当前PGA112增益编码

// 校准相关变量
static UltrasonicCalibData g_calib = {0};       // RAM中缓存的校准数据
static uint8_t g_calib_valid = 0;               // 校准数据有效标记 1=Flash数据合法 0=使用理想公式

// 界面状态机：菜单切换标记
static uint8_t g_menu_sign = 0;                // 0=主菜单 1=实时测量 2=距离校准 3=系统状态 4=程控增益

/************************* 函数声明 *************************/
// 系统初始化 + 主界面绘制
static void Init_All(void);
static void Disp_Main(void);
static void Change_Menu(uint8_t menu_sign);

// UI工具函数：局部清屏、文字绘制、按键等待
static void Clear_Work_Area(void);
static void Clear_Work_Text(void);
static void Draw_Work_Title(char *title);
static void Draw_Key_Tips(char *tip1, char *tip2);
static void Show_Text_Line(uint16_t line, char *text);
static void Show_Value_Line(uint16_t line, char *label, double value, char *format);
static void Show_Value_Only(uint16_t line, double value, char *format);
static void Show_Text_Value_Only(uint16_t line, char *text);
static void Wait_Ps2KeyRelease(uint8_t key_value);

// 超声波硬件底层驱动：定时器、外部中断、PGA增益、单次测量
static void Ultrasonic_Timer_Init(void);
static void Ultrasonic_Echo_Init(void);
static void Ultrasonic_ApplyGain(uint8_t gain_code);
static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us);
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us);
static void Ultrasonic_PrepareGain(uint8_t retry_count);
static void Ultrasonic_SetAcceptWindow(uint32_t min_us, uint32_t max_us);
static void Ultrasonic_SetTrackingWindow(void);
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us);
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us);
static void Sort_Samples(uint32_t *data, uint8_t length);

// 校准、Flash读写、时间-距离转换算法
static void Calibration_Load(void);
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib);
static uint8_t Calibration_Save(const UltrasonicCalibData *calib);
static void Calibration_SetMeasureWindow(uint16_t distance_mm);
static float Convert_Time_To_Distance(uint32_t echo_us);
static float Convert_Time_To_Distance_Default(uint32_t echo_us);

// 各菜单业务逻辑处理函数
static void MenuHandler_Measure(void);
static void MenuHandler_Calibrate(void);
static void MenuHandler_Status(void);
static void MenuHandler_PGA_Test(void);

/************************* 主函数 *************************/
/**
 * @brief 应用层主入口函数，实现状态机框架
 * @note 死循环中检测菜单状态，执行对应功能模块，并响应PS2键盘输入
 */
void User_main(void)
{
    Init_All();          // 初始化所有硬件、读取Flash校准数据
    Disp_Main();         // 绘制主菜单静态界面

    // 死循环 + 状态机架构，轮询菜单与按键
    while(1)
    {
        switch(g_menu_sign)
        {
            case 0:  // 主菜单状态：轮询按键，切换子菜单
                if(Ps2KeyValue >= KeyValue_1 && Ps2KeyValue <= KeyValue_4)
                {
                    // 根据按键值(1~4)切换到对应子菜单
                    Change_Menu((uint8_t)(Ps2KeyValue - KeyValue_0));
                }
                break;

            case 1:  // 实时测量界面
                MenuHandler_Measure();
                break;

            case 2:  // 距离校准界面
                MenuHandler_Calibrate();
                break;

            case 3:  // 系统状态查看界面
                MenuHandler_Status();
                break;

            case 4:  // PGA程控增益/PWM测试界面
                MenuHandler_PGA_Test();
                break;

            default: // 异常状态兜底：状态跑飞强制切回主菜单
                g_menu_sign = 0;
                break;
        }
        delay_ms(10);  // 短暂延时，降低CPU占用、防抖、防卡死
    }
}

/************************* 系统初始化函数 *************************/
/**
 * @brief 整体硬件初始化：LCD、PWM、定时器、外部中断、加载校准数据
 */
static void Init_All(void)
{
    LCD_Clear(Black);                  // 全屏清屏为黑色背景
    Ultrasonic_PWM_Init();             // 初始化40kHz发射PWM波形驱动(互补PWM，驱动超声波探头)
    Ultrasonic_Timer_Init();           // 初始化TIM5微秒级高精度定时器
    Ultrasonic_Echo_Init();            // 初始化回波接收外部中断 (PC0双边沿触发)
    Calibration_Load();                // 从Flash读取历史校准数据到RAM

    Ultrasonic_ApplyGain(PGA112_DEFAULT_GAIN_CODE); // 设置PGA默认增益档位
    g_gain_settle_discard = 0;                     // 清除增益稳定丢弃标记
}

/************************* 主界面显示函数 *************************/
/**
 * @brief 绘制主菜单静态UI：标题、分割线、版本、菜单选项
 */
static void Disp_Main(void)
{
    uint8_t count;

    // 顶部标题居中显示
    OS_String_Show(272, 16, 32, 1, TITLE_STR);

    // 绘制白色横竖分割线，划分界面区域 (上线、下线、左右分区线)
    LCD_Appoint_Clear(0, 64, 800, 72, White);    // 顶部横向分割线
    LCD_Appoint_Clear(0, 440, 800, 448, White);  // 底部横向分割线
    LCD_Appoint_Clear(250, 72, 252, 440, White); // 左右区域纵向分割线

    // 底部状态栏：硬件型号 + 软件版本
    OS_String_Show(32, 456, 16, 1, MODEL_VER_STR); 
    OS_String_Show(680, 456, 16, 1, USER_VER_STR); 

    // 初始化菜单选项前缀 "-" (未选中标记)
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    // 绘制左侧4个菜单文字
    OS_String_Show(60, 96, 32, 1, MENU1_CHOICE1);
    OS_String_Show(60, 160, 32, 1, MENU1_CHOICE2);
    OS_String_Show(60, 224, 32, 1, MENU1_CHOICE3);
    OS_String_Show(60, 288, 32, 1, MENU1_CHOICE4);
}

/**
 * @brief 菜单切换逻辑：更新选中箭头、刷新状态机、清除按键事件
 * @param menu_sign 目标菜单编号 (1~4)
 */
static void Change_Menu(uint8_t menu_sign)
{
    uint8_t count;

    // 先清空所有菜单项前面的选中标记
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    // 绘制新选中菜单箭头 ">"，并更新状态机变量
    if(menu_sign >= 1 && menu_sign <= MENU_CHOICE_NUM)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * menu_sign), 32, 1, ">");
        g_menu_sign = menu_sign;
    }
    else
    {
        g_menu_sign = 0;  // 非法按键，切回主菜单
        Clear_Work_Area();// 清空右侧工作区
    }

    Ps2KeyValue = KeyValue_Null;  // 消耗本次按键，防止重复触发
}

/************************* UI工具函数 *************************/
/**
 * @brief 清空右侧工作区域 (调用具体清文字函数)
 */
static void Clear_Work_Area(void) { Clear_Work_Text(); }

/**
 * @brief 用空白字符局部清屏，效率高于全局LCD_Clear，避免闪屏
 * @note 根据界面布局，分别清空标题区、9行信息区、底部两行提示区
 */
static void Clear_Work_Text(void)
{
    uint8_t line;
    // 标题区清屏 (32号字体，单行)
    OS_String_Show(280, 88, 32, 1, UI_BLANK_TEXT_32);
    // 中间多行文本区清屏 (24号字体，共9行)
    for(line = 0; line < 9U; line++) 
    { 
        OS_String_Show(280, (uint16_t)(150 + line * 30), 24, 1, UI_BLANK_TEXT_24); 
    }
    // 底部提示行清屏 (16号字体，两行)
    OS_String_Show(280, 400, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 420, 16, 1, UI_BLANK_TEXT_16);
}

/**
 * @brief 绘制右侧工作区大标题
 * @param title 标题字符串
 */
static void Draw_Work_Title(char *title) { OS_String_Show(280, 88, 32, 1, title); }

/**
 * @brief 绘制底部按键操作提示 (两行)
 * @param tip1 提示文字1 (通常左侧按键功能)
 * @param tip2 提示文字2 (通常右侧按键功能)
 */
static void Draw_Key_Tips(char *tip1, char *tip2) 
{ 
    OS_String_Show(280, 400, 16, 1, tip1); 
    OS_String_Show(280, 420, 16, 1, tip2); 
}

/**
 * @brief 按行绘制纯文本 (标签区)
 * @param line 行号 (从0开始)
 * @param text 显示文本
 */
static void Show_Text_Line(uint16_t line, char *text) 
{ 
    uint16_t y = (uint16_t)(150 + line * 30); 
    OS_String_Show(280, y, 24, 1, text); 
}

/**
 * @brief 绘制"标签: 数值"组合行
 * @param line 行号
 * @param label 标签文字
 * @param value 待显示数值
 * @param format 格式化字符串 (例如 "%06.1f")
 */
static void Show_Value_Line(uint16_t line, char *label, double value, char *format) 
{ 
    char temp[24]; 
    uint16_t y = (uint16_t)(150 + line * 30); 
    sprintf(temp, format, value); 
    OS_String_Show(280, y, 24, 1, label); 
    OS_String_Show(500, y, 24, 1, temp); 
}

/**
 * @brief 仅绘制右侧数值区域 (不带标签)
 * @param line 行号
 * @param value 数值
 * @param format 格式化串
 */
static void Show_Value_Only(uint16_t line, double value, char *format) 
{ 
    char temp[24]; 
    uint16_t y = (uint16_t)(150 + line * 30); 
    sprintf(temp, format, value); 
    OS_String_Show(500, y, 24, 1, temp); 
}

/**
 * @brief 右侧数值区域显示纯文本 (字符串)
 * @param line 行号
 * @param text 文本
 */
static void Show_Text_Value_Only(uint16_t line, char *text) 
{ 
    uint16_t y = (uint16_t)(150 + line * 30); 
    OS_String_Show(500, y, 24, 1, text); 
}

/**
 * @brief 等待按键释放，简单按键防抖+防连按
 * @param key_value 需要等待释放的按键值 (例如KeyValue_Enter)
 */
static void Wait_Ps2KeyRelease(uint8_t key_value)
{
    do
    {
        Ps2KeyValue = KeyValue_Null; // 清空按键值
        delay_ms(30);                // 等待30ms
    } while(Ps2KeyValue == key_value); // 直到按键被释放
}

/************************* 超声波硬件驱动核心层 *************************/
/**
 * @brief TIM5定时器初始化：微秒级高精度计时
 * @note 选用TIM5原因：STM32F4 32位通用定时器，计数范围0~4294967295，远距离测距不会溢出
 *       APB1时钟 = 84MHz，预分频84-1，定时器计数频率 = 1MHz → 1个计数值 = 1us
 */
static void Ultrasonic_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  // 使能TIM5时钟
    TIM_TimeBaseStructInit(&tim_base);

    tim_base.TIM_Prescaler = 84 - 1;                      // 预分频 84分频 → 1MHz
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;       // 向上计数
    tim_base.TIM_Period = 0xFFFFFFFFU;                   // 最大自动重载值，超长计时不溢出
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;            // 不分频

    TIM_TimeBaseInit(TIM5, &tim_base);
    TIM_Cmd(TIM5, ENABLE);  // 定时器持续开启，全程作为微秒计时器
}

/**
 * @brief 回波引脚 + 外部中断初始化
 * @note 引脚：PC0 浮空输入
 *       中断：EXTI0 双边沿触发(上升沿+下降沿)，用于采集完整回波脉冲宽度
 *       双边沿触发原因：需要同时获取上升沿和下降沿时间，才能计算脉冲中心(波峰)
 */
static void Ultrasonic_Echo_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);    // 使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  // 使能SYSCFG(外部中断映射)

    // PC0 配置为浮空输入 (由外部超声波模块驱动)
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio_init);

    // 将PC0引脚映射到EXTI0中断线
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

    // 外部中断配置：双边沿触发
    EXTI_StructInit(&exti_init);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // 上升沿+下降沿都触发
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
    EXTI_ClearITPendingBit(EXTI_Line0); // 清除初始中断标记

    // NVIC中断优先级配置：优先级较高，避免中断延迟引入微秒级误差
    nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
    nvic_init.NVIC_IRQChannelSubPriority = 1;        
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

/**
 * @brief 设置PGA112程控放大器增益档位
 * @param gain_code 增益编码(0~7)，对应 1,2,4,8,16,32,64,128倍
 * @note 切换增益后设置 g_gain_settle_discard 标记，丢弃首次测量值等待电路稳定
 */
static void Ultrasonic_ApplyGain(uint8_t gain_code)
{
    gain_code &= 0x07U;  // 编码范围限制，防止非法编码
    if(gain_code != g_ultrasonic_gain_code)
    {
        PGA112_SetGainCode(gain_code);               // 硬件设置增益 (通过SPI)
        g_ultrasonic_gain_code = gain_code;
        g_gain_settle_discard = 1;                   // 标记：增益切换后电路需稳定，丢弃首次测量值
    }
}

/**
 * @brief 根据历史回波时间(距离)自适应选择增益档位
 * @note 原理：超声波远距离信号衰减严重，距离越远，所需放大倍数越大
 *       根据经验阈值划分8档，保证回波信号幅度适中
 * @param echo_us 历史回波时间 (us)
 * @return PGA增益编码 (0~7)
 */
static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us)
{
    if(echo_us < 180U)       return PGA112_GAIN_1;    // 极近：不放大
    if(echo_us < 360U)       return PGA112_GAIN_2;    
    if(echo_us < 800U)       return PGA112_GAIN_4;    
    if(echo_us < 1800U)      return PGA112_GAIN_8;    
    if(echo_us < 3600U)      return PGA112_GAIN_16;   // 中远距离开始放大
    if(echo_us < 6500U)      return PGA112_GAIN_32;   
    if(echo_us < 9500U)      return PGA112_GAIN_64;   
    return PGA112_GAIN_128;                           // 极限距离：最大128倍放大
}

/**
 * @brief 计算回波脉冲中心点(波峰)时间
 * @note 原理：超声波回波经过检波后输出为方波，信号能量峰值出现在方波中心
 *       使用中心时刻替代边沿，可提升测距精度，减小脉宽带来的误差
 * @param rise_us 上升沿时间 (us)
 * @param fall_us 下降沿时间 (us)
 * @return 波峰对应的微秒时间
 */
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us)
{
    uint32_t width;
    // 异常保护：下降沿小于上升沿(定时器溢出/干扰)，直接返回上升沿
    if(fall_us <= rise_us)
    {
        return rise_us;
    }
    width = fall_us - rise_us;
    return rise_us + width / 2U; // 取脉冲中心
}

/**
 * @brief 自适应增益准备：测量失败时逐级提升增益
 * @param retry_count 失败重试次数 (0~3)
 * @note 基于上一次有效回波时间选择基础增益，若连续失败则额外增加增益档位
 */
static void Ultrasonic_PrepareGain(uint8_t retry_count)
{
    // 基于上一次有效距离选择基础增益
    uint8_t gain_code = Ultrasonic_SelectGainCode(g_last_echo_us);

    // 连续测量失败，逐级增加增益 (每次失败提升一档)
    if(retry_count > ULTRASONIC_GAIN_RETRY_MAX)
    {
        retry_count = ULTRASONIC_GAIN_RETRY_MAX;
    }
    gain_code = (uint8_t)(gain_code + retry_count);

    if(gain_code > PGA112_GAIN_128)
    {
        gain_code = PGA112_GAIN_128; // 限制最大增益
    }
    Ultrasonic_ApplyGain(gain_code);
    delay_us(20); // 等待SPI配置+运放电路稳定 (约20us)
}

/**
 * @brief 设置回波有效接收时间窗口，过滤窗口外干扰信号
 * @param min_us 窗口下限 (us)
 * @param max_us 窗口上限 (us)
 * @note 若参数无效则恢复全范围窗口
 */
static void Ultrasonic_SetAcceptWindow(uint32_t min_us, uint32_t max_us)
{
    if(max_us <= min_us)
    {
        min_us = 0;
        max_us = ULTRASONIC_TIMEOUT_US;
    }
    g_echo_accept_min_us = min_us;
    g_echo_accept_max_us = max_us;
}

/**
 * @brief 动态设置跟踪窗口：锁定目标后缩小窗口，丢失目标后全域搜索
 * @note 跟踪窗口利用目标空间相关性，在历史值附近开窗，提升抗干扰能力
 *       若连续丢失目标则扩大窗口重新捕获
 */
static void Ultrasonic_SetTrackingWindow(void)
{
    const uint32_t margin_us = ULTRASONIC_TRACK_MARGIN_US;
    if(g_reacquire_ignore_near != 0U)
    {
        // 重搜索模式：屏蔽近距离 (避开盲区)，从最小有效远距离开始搜索
        Ultrasonic_SetAcceptWindow(ULTRASONIC_REACQUIRE_MIN_US, ULTRASONIC_TIMEOUT_US);
    }
    else if(g_tracking_valid == 0U)
    {
        // 未锁定目标：全开窗口，全域搜索
        Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);
    }
    else if(g_last_echo_us > margin_us)
    {
        // 已锁定且历史值大于余量：以历史值为中心，左右扩展余量，窄窗口跟踪
        Ultrasonic_SetAcceptWindow(g_last_echo_us - margin_us, g_last_echo_us + margin_us);
    }
    else
    {
        // 近距离目标：下限设为盲区 (避免自激)，上限为历史值+余量
        Ultrasonic_SetAcceptWindow(ULTRASONIC_BLANKING_US, g_last_echo_us + margin_us);
    }
}

/**
 * @brief 单次超声波发射+回波采集 (物理层)
 * @param echo_us 输出：本次回波时间 (us)
 * @return 1=采集成功 0=超时失败
 * @note 阻塞轮询等待回波，超时时间 ULTRASONIC_TIMEOUT_US
 */
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us)
{
    uint32_t timeout;
    // 1. 复位所有中断状态标记
    g_echo_captured = 0;
    g_measure_active = 1;
    g_echo_time_us = 0;
    g_echo_rise_us = 0;
    g_echo_fall_us = 0;
    g_echo_rise_seen = 0;

    // 2. 清空定时器计数值 + 清除中断挂起 (确保从0开始计时)
    TIM_SetCounter(TIM5, 0);
    EXTI_ClearITPendingBit(EXTI_Line0);

    // 3. 驱动探头发射一串40kHz超声波脉冲串 (通常8个脉冲)
    Ultrasonic_FireBurst();

    // 4. 阻塞轮询等待回波，超时则退出
    for(timeout = 0; timeout < ULTRASONIC_TIMEOUT_US / 10U; timeout++)
    {
        if(g_echo_captured != 0U)
        {
            *echo_us = g_echo_time_us;
            g_measure_active = 0; // 关闭测量窗口
            return 1;
        }
        delay_us(10);
    }

    // 超时失败
    g_measure_active = 0;
    return 0;
}

/**
 * @brief 带多级滤波的批量测量：采样+增益自适应+聚类降噪
 * @param echo_us 输出最终有效回波时间 (us)
 * @return 1=采样成功 0=全部失败
 * @note 算法流程：
 *       1. 自适应增益调整
 *       2. 连续采集 ULTRASONIC_FILTER_SAMPLES 个有效样本
 *       3. 插入排序
 *       4. 查找最密集样本簇 (容忍 ±90us 波动)
 *       5. 取中位数作为最终结果
 */
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us)
{
    uint32_t samples[ULTRASONIC_FILTER_SAMPLES]; // 采样缓冲区
    uint8_t valid_count = 0;                     // 有效样本计数
    uint8_t attempts = 0;                        // 总发射次数 (防止死循环)
    uint8_t gain_retry = 0;                      // 增益重试计数
    uint8_t miss_count = 0;                      // 连续丢失回波计数
    uint8_t index;
    uint32_t reacquire_min_us = ULTRASONIC_REACQUIRE_MIN_US;

    // 校准数据有效时，使用第一个校准点时间作为重搜索下限 (精度更高)
    if((g_calib_valid != 0U) && (g_calib.point_us[0] > reacquire_min_us))
    {
        reacquire_min_us = g_calib.point_us[0];
    }

    // 循环采集，凑够指定数量有效样本，最多额外允许20次发射机会
    while(attempts < (ULTRASONIC_FILTER_SAMPLES + 20U) && valid_count < ULTRASONIC_FILTER_SAMPLES)
    {
        uint32_t sample = 0;
        Ultrasonic_PrepareGain(gain_retry); // 动态配置增益 (根据上次结果和失败次数)
        attempts++;

        if(Ultrasonic_MeasureOnce(&sample) != 0U)
        {
            // 增益刚切换，电路未稳定，丢弃脏数据
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
                delay_ms(8);
                continue;
            }
            // 重搜索模式下屏蔽近距离干扰 (避开盲区)
            if((g_reacquire_ignore_near != 0U) && (sample < reacquire_min_us))
            {
                continue;
            }
            // 存入有效样本
            samples[valid_count++] = sample;
            g_last_echo_us = sample;         // 更新历史值
            gain_retry = 0;                  // 成功则清空重试计数
            miss_count = 0;
            g_reacquire_ignore_near = 0U;    // 成功采集后退出重搜索模式
        }
        else
        {
            // 单次测量失败处理
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
            }
            else
            {
                // 未超限则提升增益重试 (逐级增加)
                if(gain_retry < ULTRASONIC_GAIN_RETRY_MAX)
                {
                    gain_retry++;
                }
                // 连续丢失回波，判定目标丢失，进入全域重搜索
                if(g_tracking_valid != 0U)
                {
                    miss_count++;
                    if(miss_count >= ULTRASONIC_REACQUIRE_MISSES)
                    {
                        // 重置所有状态，扩大搜索窗口
                        valid_count = 0U;
                        miss_count = 0U;
                        g_tracking_valid = 0U;
                        g_reacquire_ignore_near = 1U;
                        g_last_echo_us = ULTRASONIC_TIMEOUT_US;
                        gain_retry = ULTRASONIC_GAIN_RETRY_MAX;
                        Ultrasonic_SetAcceptWindow(reacquire_min_us, ULTRASONIC_TIMEOUT_US);
                    }
                }
            }
        }
        delay_ms(8); // 降低发射占空比，防止声波叠加形成杂波干扰
    }

    if(valid_count == 0U)
    {
        return 0; // 无有效样本，测量失败
    }

    // 样本排序 (升序)
    Sort_Samples(samples, valid_count);

    // 聚类算法：选取最密集样本簇的中间值作为最终结果，剔除跳变干扰
    if(valid_count >= 3U)
    {
        uint8_t best_start = 0U;
        uint8_t best_count = 1U;
        uint8_t best_mid;
        for(index = 0U; index < valid_count; index++)
        {
            uint8_t count = 1U;
            uint8_t scan;
            for(scan = (uint8_t)(index + 1U); scan < valid_count; scan++)
            {
                if((samples[scan] - samples[index]) <= ULTRASONIC_CLUSTER_SPAN_US)
                {
                    count++;
                }
                else
                {
                    break;
                }
            }
            if(count > best_count)
            {
                best_count = count;
                best_start = index;
            }
        }
        if(best_count < ULTRASONIC_CLUSTER_MIN_COUNT)
        {
            return 0;
        }
        {
            uint32_t sum = 0U;
            for(index = best_start; index < (uint8_t)(best_start + best_count); index++)
            {
                sum += samples[index];
            }
            *echo_us = (sum + best_count / 2U) / best_count;
        }
    }
    else
    {
        // 样本较少，直接取中间值
        *echo_us = samples[valid_count / 2U];
    }

    g_last_echo_us = *echo_us;
    return 1;
}

/**
 * @brief 插入排序：对采样数组升序排列，用于后续聚类、去极值
 * @param data 待排序数组
 * @param length 数组长度
 */
static void Sort_Samples(uint32_t *data, uint8_t length)
{
    uint8_t i;
    for(i = 1U; i < length; i++)
    {
        uint32_t key = data[i];
        int8_t j = (int8_t)i - 1;
        // 向前移位，找到插入位置
        while(j >= 0 && data[j] > key)
        {
            data[j + 1] = data[j];
            j--;
        }
        data[j + 1] = key;
    }
}

/************************* 校准与距离转换数据层 *************************/
/**
 * @brief 从Flash指定地址读取校准数据到RAM
 * @note 直接指针强制转换读取，不经过缓存，读取后自动校验合法性
 */
static void Calibration_Load(void)
{
    // 地址强制转为结构体指针，直接读取Flash数据
    const UltrasonicCalibData *stored = (const UltrasonicCalibData *)ULTRASONIC_FLASH_ADDR;
    g_calib = *stored; 
    g_calib_valid = Calibration_IsValid(&g_calib); // 校验数据合法性
}

/**
 * @brief 校验校准数据是否合法：魔术字、版本、时序逻辑校验
 * @param calib 待校验结构体
 * @return 1=合法 0=非法
 */
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib)
{
    uint8_t index;
    // 1. 魔术字+版本校验 (防止读取到随机Flash内容)
    if(calib->magic != ULTRASONIC_FLASH_MAGIC || calib->version != ULTRASONIC_FLASH_VERSION)
    {
        return 0;
    }
    // 2. 物理逻辑校验：距离越远，回波时间必须单调递增，且不超过最大量程
    for(index = 0; index < 5U; index++)
    {
        if(calib->point_us[index] == 0U || calib->point_us[index] > ULTRASONIC_TIMEOUT_US)
        {
            return 0; // 时间超出合法范围
        }
        if(index > 0U && calib->point_us[index] <= calib->point_us[index - 1U])
        {
            return 0; // 非单调递增 (物理上不可能)
        }
    }
    return 1;  
}

/**
 * @brief 将校准数据写入Flash Sector11
 * @param calib 待写入校准结构体
 * @return 1=写入成功 0=失败
 * @note Flash特性：只能擦除(写1)和编程(写0)，不能单独修改字节；擦除按扇区操作
 *       Sector11大小128KB，地址范围 0x080E0000 - 0x080FFFFF
 */
static uint8_t Calibration_Save(const UltrasonicCalibData *calib)
{
    FLASH_Status status = FLASH_COMPLETE;
    const uint32_t *words = (const uint32_t *)calib; 
    uint32_t address = ULTRASONIC_FLASH_ADDR;
    uint32_t index;

    FLASH_Unlock(); // 解锁Flash写保护
    // 清除Flash错误标记 (清理上一步可能产生的错误)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 整扇区擦除 (所有位变为1)
    status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    if(status == FLASH_COMPLETE)
    {
        // 按32bit字逐字写入 (结构体大小32字节，共8个字)
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
    FLASH_Lock(); // 重新上锁保护

    return (uint8_t)(status == FLASH_COMPLETE);
}

/**
 * @brief 校准时根据标准距离设置专用接收窗口，提升标定精度
 * @param distance_mm 当前校准标准距离 (mm)
 * @note 通过限制窗口范围，避免校准时采集到多路径干扰或邻近物体回波
 */
static void Calibration_SetMeasureWindow(uint16_t distance_mm)
{
    switch(distance_mm)
    {
        case 100U:
            Ultrasonic_SetAcceptWindow(ULTRASONIC_BLANKING_US, 1800U);
            break;
        case 300U:
            Ultrasonic_SetAcceptWindow(1200U, 3200U);
            break;
        case 600U:
            Ultrasonic_SetAcceptWindow(3000U, 5600U);
            break;
        case 900U:
            Ultrasonic_SetAcceptWindow(5000U, 7800U);
            break;
        case 1300U:
            Ultrasonic_SetAcceptWindow(7400U, ULTRASONIC_TIMEOUT_US);
            break;
        default:
            Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);
            break;
    }
}

/**
 * @brief 无校准数据时，使用理想物理公式计算距离 (兜底方案)
 * @param echo_us 回波时间(us)
 * @return 距离(mm)
 * @note 理论公式：距离(mm) = 时间(us) * 0.1715
 *       实际使用 0.164866 系数适配HC-SR04模块常见偏差
 */
static float Convert_Time_To_Distance_Default(uint32_t echo_us)
{
    float distance = (float)echo_us * 0.164866f;
    // 限幅到量程范围 10mm ~ 1300mm
    if(distance < 10.0f) distance = 10.0f;
    if(distance > 1300.0f) distance = 1300.0f;
    return distance;
}

/**
 * @brief 分段线性插值距离转换 (使用校准数据，修正系统误差)
 * @param echo_us 回波时间(us)
 * @return 修正后距离(mm)
 * @note 原理：将全量程分为多段直线，用标定点拟合非线性误差
 *       本例使用5个标定点，分成4段直线，两点式插值
 */
static float Convert_Time_To_Distance(uint32_t echo_us)
{
    uint8_t index;
    float x0, x1;  // 横坐标：实际测量回波时间 (us)
    float y0, y1;  // 纵坐标：标准物理距离 (mm)
    float distance;

    if(g_calib_valid == 0U)
    {
        return Convert_Time_To_Distance_Default(echo_us); // 无校准则使用理想公式
    }

    // 根据回波时间判断落在哪一段折线区间内
    index = 0U;
    while((index < 3U) && (echo_us > g_calib.point_us[index + 1U]))
    {
        index++;
    }

    x0 = (float)g_calib.point_us[index];
    x1 = (float)g_calib.point_us[index + 1U];
    y0 = (float)k_calib_distance_mm[index];
    y1 = (float)k_calib_distance_mm[index + 1U];

    if(x1 <= x0) // 异常防护
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    // 两点式直线插值: y = y0 + (x - x0)*(y1 - y0)/(x1 - x0)
    distance = y0 + ((float)echo_us - x0) * (y1 - y0) / (x1 - x0);

    // 限幅到有效量程
    if(distance < (float)k_calib_distance_mm[0]) distance = (float)k_calib_distance_mm[0];
    if(distance > 1300.0f) distance = 1300.0f;
    return distance;
}

/************************* 菜单交互业务层 *************************/
/**
 * @brief 手动测量界面业务逻辑
 * @note 按下一次Enter执行一次滤波测距，显示时间、距离、校准状态、增益等信息
 *       按下返回键退回主菜单
 */
static void MenuHandler_Measure(void)
{
    char value_text[24];
    Draw_Work_Title("手动测量");
    Draw_Key_Tips("Enter: 测量一次", "Back: 返回");

    g_tracking_valid = 0;
    g_reacquire_ignore_near = 0U;

    // 绘制固定文本(标签)
    OS_String_Show(280, 150, 24, 1, "测量时间(us)");
    OS_String_Show(280, 180, 24, 1, "测量距离(mm)");
    OS_String_Show(280, 210, 24, 1, "默认距离(mm)");
    OS_String_Show(280, 240, 24, 1, "校准状态");
    OS_String_Show(280, 270, 24, 1, "前端增益(x)");
    OS_String_Show(280, 300, 24, 1, "提示信息");

    Show_Text_Value_Only(0, "-----");
    Show_Text_Value_Only(1, "------");
    Show_Text_Value_Only(2, "------");
    Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "已校准" : "未校准");
    Show_Text_Value_Only(4, "008");
    Show_Text_Value_Only(5, "等待Enter");

    // 手动测量：每次按下Enter仅执行一次多采样滤波测量，结果保留至下次触发
    while(1)
    {
        uint32_t echo_us = 0;

        if(Ps2KeyValue == KeyValue_Back)
        {
            break;
        }

        if(Ps2KeyValue == KeyValue_Enter)
        {
            Ps2KeyValue = KeyValue_Null;
            Show_Text_Value_Only(5, "正在测量  ");

            Ultrasonic_SetTrackingWindow(); // 根据跟踪状态设置接收窗口

            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                float distance = Convert_Time_To_Distance(echo_us);
                g_tracking_valid = 1; // 测距成功，标记跟踪功能有效

                sprintf(value_text, "%05lu", (unsigned long)echo_us);
                Show_Text_Value_Only(0, value_text);

                sprintf(value_text, "%06.1f", (double)distance);
                Show_Text_Value_Only(1, value_text);

                sprintf(value_text, "%06.1f", (double)Convert_Time_To_Distance_Default(echo_us));
                Show_Text_Value_Only(2, value_text);

                Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "已校准  " : "未校准  ");
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(5, "测量完成  ");
            }
            else
            {
                // 测量失败：进入重搜索模式
                g_reacquire_ignore_near = 1U;
                g_tracking_valid = 0;
                Show_Text_Value_Only(3, "测量失败  ");
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(5, "检查探头  ");
            }
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }

        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0); // 返回主菜单
}

/**
 * @brief 超声波校准菜单处理函数
 * @details 分步完成5个标准距离点采样、合法性校验、校准数据写入Flash
 *          支持确认键采样、返回键退出校准流程
 * @note 校准步骤：
 *       1. 在指定距离放置反射板 (依次 100,300,600,900,1300mm)
 *       2. 按下确认键自动采样
 *       3. 全部5个点位完成后，自动校验并保存数据到Flash
 */
static void MenuHandler_Calibrate(void)
{
    UltrasonicCalibData new_calib;
    uint8_t step = 0;
    uint8_t last_step = 0xFFU;
    char line[24];

    // 初始化校准数据结构体 (写入魔术字、版本号，清空时间点位)
    new_calib.magic = ULTRASONIC_FLASH_MAGIC;
    new_calib.version = ULTRASONIC_FLASH_VERSION;
    new_calib.point_us[0] = 0; new_calib.point_us[1] = 0;
    new_calib.point_us[2] = 0; new_calib.point_us[3] = 0;
    new_calib.point_us[4] = 0; new_calib.reserved[0] = 0;

    Draw_Work_Title("距离校准");
    Draw_Key_Tips("确认开始校准", "返回退出校准");
    OS_String_Show(280, 150, 24, 1, "校准提示");
    OS_String_Show(280, 180, 24, 1, "当前状态");
    OS_String_Show(280, 210, 24, 1, "100mm(us)");
    OS_String_Show(280, 240, 24, 1, "300mm(us)");
    OS_String_Show(280, 270, 24, 1, "600mm(us)");
    OS_String_Show(280, 300, 24, 1, "900mm(us)");
    OS_String_Show(280, 330, 24, 1, "1300mm(us)");
    OS_String_Show(280, 360, 24, 1, "当前测量值(us)");
    OS_String_Show(280, 390, 24, 1, "校准结果");
    Show_Text_Value_Only(1, "等待校准");
    Show_Text_Value_Only(2, "00000"); Show_Text_Value_Only(3, "00000");
    Show_Text_Value_Only(4, "00000"); Show_Text_Value_Only(5, "00000");
    Show_Text_Value_Only(6, "00000"); Show_Text_Value_Only(7, "00000");
    Show_Text_Value_Only(8, "等待校准");

    while(Ps2KeyValue != KeyValue_Back)
    {
        if(step != last_step)
        {
            sprintf(line, "对标%04umm", k_calib_distance_mm[step]);
            Show_Text_Value_Only(0, line);
            last_step = step;
        }

        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;

            Show_Text_Value_Only(0, "开始校准");
            Show_Text_Value_Only(1, "正在校准");
            Calibration_SetMeasureWindow(k_calib_distance_mm[step]); // 设置测量窗口
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                new_calib.point_us[step] = echo_us;
                sprintf(line, "%05lu", (unsigned long)echo_us);
                Show_Text_Value_Only(7, line);
                Show_Text_Value_Only((uint16_t)(2 + step), line);
                Show_Text_Value_Only(1, "采样完成");
                step++;

                if(step >= 5U) // 5个点位全部采集完成
                {
                    uint8_t valid_ok = Calibration_IsValid(&new_calib);
                    uint8_t save_ok = 0U;

                    if(valid_ok != 0U)
                    {
                        save_ok = Calibration_Save(&new_calib);
                    }

                    if(valid_ok != 0U && save_ok != 0U)
                    {
                        g_calib = new_calib;
                        g_calib_valid = 1;
                        Show_Text_Value_Only(8, "校准完成  ");
                    }
                    else if(valid_ok == 0U)
                    {
                        Show_Text_Value_Only(1, "点位异常");
                        Show_Text_Value_Only(8, "POINT ERR ");
                    }
                    else
                    {
                        Show_Text_Value_Only(1, "写入失败");
                        Show_Text_Value_Only(8, "FLASH ERR ");
                    }
                    delay_ms(1000);
                    break;
                }
            }
            else
            {
                Show_Text_Value_Only(1, "校准超时");
                Show_Text_Value_Only(8, "校准失败");
            }
            Show_Text_Value_Only(8, "松开确认键");
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);
    Change_Menu(0);
}

/**
 * @brief 系统状态查看菜单处理函数
 * @details 展示校准有效性、5个校准点原始时间值、当前增益
 *          按下确认键执行一次实时测量并显示结果
 */
static void MenuHandler_Status(void)
{
    char value_text[24];

    Draw_Work_Title("实时测量");
    Draw_Key_Tips("自动测量中", "返回: 退出");

    g_tracking_valid = 0;
    g_reacquire_ignore_near = 0U;
    Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);

    OS_String_Show(280, 150, 24, 1, "回波时间(us)");
    OS_String_Show(280, 180, 24, 1, "测量距离(mm)");
    OS_String_Show(280, 210, 24, 1, "默认距离(mm)");
    OS_String_Show(280, 240, 24, 1, "校准状态");
    OS_String_Show(280, 270, 24, 1, "当前增益(x)");
    OS_String_Show(280, 300, 24, 1, "运行状态");

    Show_Text_Value_Only(0, "-----   ");
    Show_Text_Value_Only(1, "------  ");
    Show_Text_Value_Only(2, "------  ");
    Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "已校准   " : "未校准   ");
    Show_Text_Value_Only(4, "008     ");
    Show_Text_Value_Only(5, "运行中   ");

    while(Ps2KeyValue != KeyValue_Back)
    {
        uint32_t echo_us = 0;

        Ultrasonic_SetTrackingWindow();
        if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
        {
            float distance = Convert_Time_To_Distance(echo_us);
            g_tracking_valid = 1;

            sprintf(value_text, "%05lu   ", (unsigned long)echo_us);
            Show_Text_Value_Only(0, value_text);

            sprintf(value_text, "%06.1f  ", (double)distance);
            Show_Text_Value_Only(1, value_text);

            sprintf(value_text, "%06.1f  ", (double)Convert_Time_To_Distance_Default(echo_us));
            Show_Text_Value_Only(2, value_text);

            Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "已校准   " : "未校准   ");
            sprintf(value_text, "%03u     ", PGA112_GetGainValue(g_ultrasonic_gain_code));
            Show_Text_Value_Only(4, value_text);
            Show_Text_Value_Only(5, "正常     ");
        }
        else
        {
            g_reacquire_ignore_near = 1U;
            g_tracking_valid = 0;
            sprintf(value_text, "%03u     ", PGA112_GetGainValue(g_ultrasonic_gain_code));
            Show_Text_Value_Only(4, value_text);
            Show_Text_Value_Only(5, "信号异常 ");
        }

        delay_ms(60);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

/**
 * @brief PGA112程控增益测试菜单
 * @details 开启超声波PWM发射(40kHz互补PWM)，通过加减按键切换PGA112增益档位(1~128倍)
 *          按下返回键关闭PWM并退出当前菜单，用于前端电路调试
 */
static void MenuHandler_PGA_Test(void)
{
    uint8_t gain_index = 3U;  // 默认增益索引，对应8倍增益

    // PGA112 8档增益编码：1/2/4/8/16/32/64/128倍
    const uint8_t gain_codes[8] =
    {
        PGA112_GAIN_1, PGA112_GAIN_2, PGA112_GAIN_4, PGA112_GAIN_8,
        PGA112_GAIN_16, PGA112_GAIN_32, PGA112_GAIN_64, PGA112_GAIN_128
    };

    char value_text[24];  // 字符串缓冲区

    Ultrasonic_PWM_OutputEnable();                     // 开启超声波PWM输出
    Ultrasonic_ApplyGain(gain_codes[gain_index]);      // 加载初始增益

    // 界面标题、按键提示、固定文本
    Draw_Work_Title("程控增益调节");
    Draw_Key_Tips("+/-调节增益", "Back返回并关闭PWM");
    OS_String_Show(280, 150, 24, 1, "PWM输出状态");
    OS_String_Show(280, 180, 24, 1, "输出模式");
    OS_String_Show(280, 210, 24, 1, "输出频率(Hz)");
    OS_String_Show(280, 240, 24, 1, "输出占空比(%)");
    OS_String_Show(280, 270, 24, 1, "当前增益(x)");
    OS_String_Show(280, 300, 24, 1, "增益档位");
    OS_String_Show(280, 330, 24, 1, "波形说明");
    OS_String_Show(280, 360, 24, 1, "当前提示");

    // 初始化界面固定内容
    Show_Text_Value_Only(0, "开启");
    Show_Text_Value_Only(1, "互补PWM");
    Show_Text_Value_Only(2, "040000");
    Show_Text_Value_Only(3, "050");
    sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
    Show_Text_Value_Only(4, value_text);
    Show_Text_Value_Only(5, "1/2/4/8/16/32/64/128");
    Show_Text_Value_Only(6, "PD12/PD13输出");
    Show_Text_Value_Only(7, "等待调节");

    // 主循环：按键调节增益，返回键退出
    while(Ps2KeyValue != KeyValue_Back)
    {
        // 增加增益
        if(Ps2KeyValue == KeyValue_Add)
        {
            Ps2KeyValue = KeyValue_Null;
            if(gain_index < 7U) // 未到最大档位
            {
                gain_index++;
                Ultrasonic_ApplyGain(gain_codes[gain_index]); // 设置新增益
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(7, "增益已调大");
            }
            else
            {
                Show_Text_Value_Only(7, "已达最大增益");
            }
        }
        // 减小增益
        else if(Ps2KeyValue == KeyValue_Minus)
        {
            Ps2KeyValue = KeyValue_Null;
            if(gain_index > 0U) // 未到最小档位
            {
                gain_index--;
                Ultrasonic_ApplyGain(gain_codes[gain_index]); // 设置新增益
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(7, "增益已调小");
            }
            else
            {
                Show_Text_Value_Only(7, "已达最小增益");
            }
        }
        delay_ms(20);
    }

    Ultrasonic_PWM_OutputDisable();  // 关闭PWM输出
    Ps2KeyValue = KeyValue_Null;     // 清空按键状态
    Change_Menu(0);                  // 返回主菜单
}

/************************* 中断服务函数 *************************/

/**
 * @brief EXTI0 外部中断服务函数
 * @note 回波信号(PC0)边沿触发中断，采用**状态机**采集超声波回波时序
 * @流程 盲区过滤 → 采集上升沿 → 采集下降沿 → 脉冲合法性校验 → 标记采集完成
 * @attention 该中断优先级较高，需快速处理，避免阻塞其他中断
 */
void EXTI0_IRQHandler(void)
{
    // 判断是否为 EXTI_Line0 中断触发
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 仅在测量窗口开启时响应回波信号，屏蔽杂波干扰
        if(g_measure_active != 0U)
        {
            // 读取TIM5定时器数值(微秒计时)
            uint32_t now = TIM_GetCounter(TIM5);

            // 阶段1：过滤发射近端盲区，消除发射余振干扰(探头发射后的残留振荡)
            if(now >= ULTRASONIC_BLANKING_US)
            {
                // 引脚为高电平：判定为【上升沿】
                if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) != Bit_RESET)
                {
                    // 未记录上升沿 + 在有效时间窗口内，记录上升沿时刻
                    if((g_echo_rise_seen == 0U) && (now >= g_echo_accept_min_us))
                    {
                        g_echo_rise_us = now;     // 保存上升沿时间戳
                        g_echo_rise_seen = 1U;    // 标记已采集上升沿
                    }
                }
                // 引脚为低电平 + 已采集上升沿 + 时间合法：判定为【下降沿】
                else if(g_echo_rise_seen != 0U && now > g_echo_rise_us)
                {
                    g_echo_fall_us = now;  // 保存下降沿时间戳
                    // 计算回波峰值等效时间(取脉冲中心)
                    g_echo_time_us = Ultrasonic_EstimatePeakTime(g_echo_rise_us, g_echo_fall_us);

                    // 多重合法性校验：时间范围、脉冲宽度、窗口范围，过滤毛刺/干扰
                    if((g_echo_time_us >= ULTRASONIC_MIN_VALID_US) &&
                       ((g_echo_fall_us - g_echo_rise_us) >= ULTRASONIC_MIN_PULSE_WIDTH_US) &&
                       (g_echo_time_us >= g_echo_accept_min_us) &&
                       (g_echo_time_us <= g_echo_accept_max_us))
                    {
                        g_echo_captured = 1U;    // 标记采集完成，主循环可取数
                        g_measure_active = 0U;   // 关闭本次测量窗口，停止接收中断
                    }
                    // 校验失败：清空状态，等待下一次回波
                    else
                    {
                        g_echo_rise_seen = 0U;
                        g_echo_rise_us = 0U;
                        g_echo_fall_us = 0U;
                    }
                }
            }
        }

        // 必须清除中断挂起标记，否则会重复进中断
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
