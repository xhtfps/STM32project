#include "User.h"
#include "Drive_PWM.h"

/************************* 瓒呭０娉㈡ā鍧楀弬鏁伴厤缃� *************************/
/*
 * 鐗╃悊鍘熺悊锛氬父娓╃┖姘斾腑澹伴€� 鈮� 343 m/s
 * 1渭s 澹版尝鍗曠▼璺濈锛�343 * 100 / 1000000 = 0.0343 cm = 0.343 mm
 * 瓒呭０娉㈠線杩旀祴璺濓紝鍥犳 1渭s 瀵瑰簲瀹為檯璺濈锛�0.343 / 2 = 0.1715 mm
 */
// 娴嬮噺瓒呮椂鏃堕棿(渭s)锛氬搴旀渶澶ф祴璺濈害2m锛岀悊璁哄線杩旀椂闂粹増11662渭s锛岄鐣欎綑閲忚涓�12000渭s
#define ULTRASONIC_TIMEOUT_US        12000U
// 鏈€灏忔湁鏁堝洖娉㈡椂闂�(渭s)锛氭护闄ょ數璺覆鎵般€侀潤鐢典骇鐢熺殑鏋佺煭骞叉壈鑴夊啿
#define ULTRASONIC_MIN_VALID_US      20U
// 鍥炴尝鑴夊啿鏈€灏忓搴�(渭s)锛氬皬浜庤鍊煎垽瀹氫负姣涘埡骞叉壈锛屼涪寮�
#define ULTRASONIC_MIN_PULSE_WIDTH_US  30U
// 鍙戝皠鐩插尯/娑堥殣鏃堕棿(渭s)锛氭帰澶村彂灏勫悗瀛樺湪鏈烘浣欓渿+鐢佃矾鑷縺锛屾娈垫椂闂村睆钄芥帴鏀�
// 450渭s鐩插尯瀵瑰簲杩戣窛绂荤害 7.7cm锛岄伩鍏嶈繎璺濊嚜婵€璇Е鍙�
#define ULTRASONIC_BLANKING_US       450U   
// 婊ゆ尝閲囨牱娆℃暟锛氬崟娆℃湁鏁堟祴璺濋噰闆哊缁勬牱鏈紝閰嶅悎鍘绘瀬鍊笺€佽仛绫荤畻娉曢檷鍣�
#define ULTRASONIC_FILTER_SAMPLES    60U
// 鏍锋湰鑱氱被鍖洪棿(渭s)锛氬垽瀹氫袱缁勯噰鏍峰€兼槸鍚﹀睘浜庡悓涓€鏈夋晥鍥炴尝绨�
#define ULTRASONIC_CLUSTER_SPAN_US   180U
// 澧炵泭鏈€澶ч噸璇曟鏁帮細鍥炴尝寰急鏈娴嬪埌鏃讹紝閫愮骇鎶崌PGA澧炵泭鐨勬渶澶у皾璇曟鏁�
#define ULTRASONIC_GAIN_RETRY_MAX    3U
// 璺熻釜绐楀彛浣欓噺(渭s)锛氶攣瀹氭湁鏁堝洖娉㈠悗锛屽湪鍘嗗彶鍊煎熀纭€涓婃墿灞曠獥鍙ｈ寖鍥达紝鍔ㄦ€佽窡韪洰鏍�
#define ULTRASONIC_TRACK_MARGIN_US   3000U
// 杩炵画涓㈠け鍥炴尝娆℃暟闃堝€硷細瓒呰繃璇ュ€煎垽瀹氱洰鏍囦涪澶憋紝閲嶆柊鍏ㄥ煙鎼滅储
#define ULTRASONIC_REACQUIRE_MISSES  2U
// 閲嶆悳绱㈡ā寮忎笅鏈€灏忔帴鏀舵椂闂�(渭s)锛氬睆钄借繎璺濈鑷縺骞叉壈锛屼笓娉ㄨ繙璺濈鎼滅储
#define ULTRASONIC_REACQUIRE_MIN_US  650U
// 鏍″噯鏁版嵁Flash瀛樺偍鍦板潃锛歋TM32F407 Sector11璧峰鍦板潃(Flash鏈€鍚�128KB鎵囧尯)
// 閫夌敤鏈熬鎵囧尯锛岄伩鍏嶄笌绋嬪簭浠ｇ爜鍖�(0x08000000寮€濮�)鍐茬獊
#define ULTRASONIC_FLASH_ADDR        0x080E0000U
// Flash鏁版嵁榄旀暟锛欰SCII "USON" (0x55 0x53 0x4F 0x4E)
// 涓婄數鏍￠獙榄旀暟锛屽垽鏂璅lash鍖哄煙鏄惁涓哄悎娉曟牎鍑嗘暟鎹紝鍖哄垎绌虹櫧/涔辩爜
#define ULTRASONIC_FLASH_MAGIC       0x55534F4EU  
// 鏁版嵁鐗堟湰鍙凤細缁撴瀯浣�/瀛楁鍙樻洿鏃跺崌绾х増鏈紝浣挎棫鐗堟牎鍑嗘暟鎹嚜鍔ㄥけ鏁堬紝闃叉瑙ｆ瀽閿欒
#define ULTRASONIC_FLASH_VERSION     0x00010004U  

/************************* 灞忓箷鏄剧ず甯搁噺瀹氫箟 *************************/
#define TITLE_STR        "瓒呭０娉㈡祴璺濅华"         // 涓荤晫闈㈤《閮ㄦ爣棰�
#define MODEL_VER_STR    "鍨嬪彿锛欻C-SR04"       // 纭欢鍨嬪彿鏍囨敞
#define USER_VER_STR     "鐗堟湰锛歏1.0"          // 杞欢鐗堟湰鍙�
#define MENU1_CHOICE1    "1. 鎵嬪姩娴嬮噺"         // 鑿滃崟閫夐」1
#define MENU1_CHOICE2    "2. 璺濈鏍″噯"         // 鑿滃崟閫夐」2
#define MENU1_CHOICE3    "3. 绯荤粺鐘舵€�"         // 鑿滃崟閫夐」3
#define MENU1_CHOICE4    "4. 绋嬫帶璋冭妭"         // 鑿滃崟閫夐」4
#define MENU_CHOICE_NUM  4                    // 鑿滃崟鎬绘暟閲�

// 灞€閮ㄦ竻灞忕┖鐧戒覆锛氱敤绌烘牸瑕嗙洊鍘熸湁瀛楃锛屾瘮鍏ㄥ眬娓呭睆LCD_Clear鏁堢巼鏇撮珮锛屽噺灏戝埛灞忛棯鐑�
#define UI_BLANK_TEXT_16 "                                                                "  // 16鍙峰瓧浣� 64瀛楃绌烘牸
#define UI_BLANK_TEXT_24 "                                                "                // 24鍙峰瓧浣� 48瀛楃绌烘牸
#define UI_BLANK_TEXT_32 "                                "                                // 32鍙峰瓧浣� 32瀛楃绌烘牸
#define UI_VALUE_BLANK_24 "                        "                                       // 鏁板€煎尯涓撶敤绌虹櫧涓�

/************************* 鏁版嵁缁撴瀯瀹氫箟 *************************/

/**
 * @brief 瓒呭０娉㈠垎娈电嚎鎬ф牎鍑嗘暟鎹粨鏋勪綋
 * @note 鐢ㄤ簬淇纭欢寤惰繜銆佸０閫熸紓绉汇€佺數璺潪绾挎€ц宸紝閲囩敤澶氱偣鍒嗘鎻掑€兼牎鍑�
 * @attention 缁撴瀯浣撹嚜鐒�4瀛楄妭瀵归綈锛岄€傞厤Flash 32bit瀛楀啓鍏ヨ鍒�
 */
typedef struct
{
    uint32_t magic;               // 鏁版嵁鍚堟硶鎬ч瓟鏁版爣蹇�
    uint32_t version;             // 缁撴瀯浣撶増鏈彿
    uint32_t point_us[5];         // 瀛樺偍5涓爣鍑嗚窛绂讳笅瀹為檯娴嬪緱鐨勫洖娉㈡椂闂�(渭s)
    uint32_t reserved[1];         // 棰勭暀绌洪棿锛屼繚璇佺粨鏋勪綋鎬诲ぇ灏�32瀛楄妭
} UltrasonicCalibData;

// 5缁勫熀鍑嗘牎鍑嗚窛绂�(鍗曚綅锛歮m)锛岃鐩栬繎銆佷腑銆佽繙鍏ㄩ噺绋嬪尯闂�
static const uint16_t k_calib_distance_mm[5] = {100, 300, 600, 900, 1300};

/************************* 鍏ㄥ眬鍙橀噺瀹氫箟 *************************/
/*
 * volatile 鍏抽敭瀛楄鏄庯細
 * 浠ヤ笅鍙橀噺鍧囧湪銆愬閮ㄤ腑鏂璄XTI銆戜腑淇敼锛寁olatile寮哄埗缂栬瘧鍣ㄦ瘡娆′粠鍐呭瓨璇诲彇锛�
 * 绂佹缂栬瘧鍣ㄤ紭鍖栵紝闃叉涓柇涓庝富寰幆鍙橀噺涓嶅悓姝ャ€佺姸鎬佹満鍗℃銆�
 */
static volatile uint8_t g_echo_captured = 0;    // 鍥炴尝鎹曡幏瀹屾垚鏍囧織 1=鎴愬姛 0=鏈畬鎴�
static volatile uint8_t g_measure_active = 0;   // 娴嬮噺绐楀彛浣胯兘鏍囧織 1=鍏佽涓柇鎺ユ敹鍥炴尝 0=灞忚斀
static volatile uint32_t g_echo_time_us = 0;    // 鏈€缁堣绠楀緱鍒扮殑鍥炴尝宄板€兼椂闂�(渭s)
static volatile uint32_t g_echo_rise_us = 0;    // 鍥炴尝鑴夊啿涓婂崌娌挎椂鍒�(渭s)
static volatile uint32_t g_echo_fall_us = 0;    // 鍥炴尝鑴夊啿涓嬮檷娌挎椂鍒�(渭s)
static volatile uint8_t g_echo_rise_seen = 0;   // 鐘舵€佹満鏍囧織锛氭槸鍚﹀凡妫€娴嬪埌涓婂崌娌�
static volatile uint32_t g_echo_accept_min_us = 0;  // 鍥炴尝鎺ユ敹绐楀彛涓嬮檺(渭s)
static volatile uint32_t g_echo_accept_max_us = ULTRASONIC_TIMEOUT_US; // 鍥炴尝鎺ユ敹绐楀彛涓婇檺(渭s)

static uint8_t g_gain_settle_discard = 0;       // 澧炵泭鍒囨崲鏍囧織锛�1=涓㈠純鏈娴嬮噺(杩愭斁鐢佃矾闇€绋冲畾鏃堕棿)
static uint32_t g_last_echo_us = 1500U;         // 涓婁竴娆℃湁鏁堝洖娉㈡椂闂达紝鐢ㄤ簬棰勬祴褰撳墠澧炵泭妗ｄ綅
static uint8_t g_tracking_valid = 0;           // 璺熻釜绐楀彛鏍囧織 1=宸查攣瀹氭湁鏁堝洖娉紝寮€鍚獎绐楀彛璺熻釜
static uint8_t g_reacquire_ignore_near = 0;     // 閲嶆悳绱㈡爣蹇� 1=灞忚斀杩戣窛绂讳俊鍙凤紝涓撴敞杩滆窛绂绘悳绱�
static uint8_t g_ultrasonic_gain_code = PGA112_DEFAULT_GAIN_CODE; // 褰撳墠PGA112澧炵泭缂栫爜

// 鏍″噯鐩稿叧鍙橀噺
static UltrasonicCalibData g_calib = {0};       // RAM涓紦瀛樼殑鏍″噯鏁版嵁
static uint8_t g_calib_valid = 0;               // 鏍″噯鏁版嵁鏈夋晥鏍囧織 1=Flash鏁版嵁鍚堟硶 0=浣跨敤鐞嗘兂鍏紡

// 鐣岄潰鐘舵€佹満锛氳彍鍗曞垏鎹㈡爣蹇�
static uint8_t g_menu_sign = 0;                // 0=涓昏彍鍗� 1=瀹炴椂娴嬮噺 2=璺濈鏍″噯 3=绯荤粺鐘舵€� 4=绋嬫帶澧炵泭

/************************* 鍑芥暟澹版槑 *************************/
// 绯荤粺鍒濆鍖� + 涓荤晫闈㈢粯鍒�
static void Init_All(void);
static void Disp_Main(void);
static void Change_Menu(uint8_t menu_sign);

// UI宸ュ叿鍑芥暟锛氬眬閮ㄦ竻灞忋€佹枃瀛楃粯鍒躲€佹寜閿瓑寰�
static void Clear_Work_Area(void);
static void Clear_Work_Text(void);
static void Draw_Work_Title(char *title);
static void Draw_Key_Tips(char *tip1, char *tip2);
static void Show_Text_Line(uint16_t line, char *text);
static void Show_Value_Line(uint16_t line, char *label, double value, char *format);
static void Show_Value_Only(uint16_t line, double value, char *format);
static void Show_Text_Value_Only(uint16_t line, char *text);
static void Wait_Ps2KeyRelease(uint8_t key_value);

// 瓒呭０娉㈢‖浠跺簳灞傞┍鍔細瀹氭椂鍣ㄣ€佸閮ㄤ腑鏂€丳GA澧炵泭銆佸崟娆℃祴閲�
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

// 鏍″噯銆丗lash璇诲啓銆佹椂闂�-璺濈杞崲绠楁硶
static void Calibration_Load(void);
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib);
static uint8_t Calibration_Save(const UltrasonicCalibData *calib);
static void Calibration_SetMeasureWindow(uint16_t distance_mm);
static float Convert_Time_To_Distance(uint32_t echo_us);
static float Convert_Time_To_Distance_Default(uint32_t echo_us);

// 鍚勮彍鍗曚笟鍔￠€昏緫澶勭悊鍑芥暟
static void MenuHandler_Measure(void);
static void MenuHandler_Calibrate(void);
static void MenuHandler_Status(void);
static void MenuHandler_PGA_Test(void);

/************************* 涓诲嚱鏁� *************************/
/**
 * @brief 搴旂敤灞備富鍏ュ彛鍑芥暟锛岃８鏈虹姸鎬佹満妗嗘灦
 * @note 鏃犻檺寰幆涓娴嬭彍鍗曠姸鎬侊紝鎵ц瀵瑰簲鍔熻兘妯″潡锛屽苟鍝嶅簲PS2閿洏杈撳叆
 */
void User_main(void)
{
    Init_All();          // 鍒濆鍖栨墍鏈夌‖浠躲€佽鍙朏lash鏍″噯鏁版嵁
    Disp_Main();         // 缁樺埗涓昏彍鍗曢潤鎬佺晫闈�

    // 瑁告満姝诲惊鐜� + 鐘舵€佹満鏋舵瀯锛岃疆璇㈣彍鍗曚笌鎸夐敭
    while(1)
    {
        switch(g_menu_sign)
        {
            case 0:  // 涓昏彍鍗曠姸鎬侊細杞鎸夐敭锛屽垏鎹㈠瓙鑿滃崟
                if(Ps2KeyValue >= KeyValue_1 && Ps2KeyValue <= KeyValue_4)
                {
                    // 鏍规嵁鎸夐敭鍊�(1~4)鍒囨崲鍒板搴斿瓙鑿滃崟
                    Change_Menu((uint8_t)(Ps2KeyValue - KeyValue_0));
                }
                break;

            case 1:  // 瀹炴椂娴嬮噺鐣岄潰
                MenuHandler_Measure();
                break;

            case 2:  // 璺濈鏍″噯鐣岄潰
                MenuHandler_Calibrate();
                break;

            case 3:  // 绯荤粺鐘舵€佹煡鐪嬬晫闈�
                MenuHandler_Status();
                break;

            case 4:  // PGA绋嬫帶澧炵泭/PWM娴嬭瘯鐣岄潰
                MenuHandler_PGA_Test();
                break;

            default: // 寮傚父鐘舵€佸厹搴曪細鐘舵€佽窇椋炲己鍒跺垏鍥炰富鑿滃崟
                g_menu_sign = 0;
                break;
        }
        delay_ms(10);  // 鐭殏寤舵椂锛岄檷浣嶤PU鍗犵敤銆佹秷鎶栥€侀槻绌鸿浆
    }
}

/************************* 绯荤粺鍒濆鍖栧嚱鏁� *************************/
/**
 * @brief 鏁翠綋纭欢鍒濆鍖栵細LCD銆丳WM銆佸畾鏃躲€佸閮ㄤ腑鏂€佹牎鍑嗘暟鎹姞杞�
 */
static void Init_All(void)
{
    LCD_Clear(Black);                  // 鍏ㄥ睆娓呭睆涓洪粦鑹茶儗鏅�
    Ultrasonic_PWM_Init();             // 鍒濆鍖�40kHz鍙戝皠PWM娉㈠舰椹卞姩 (浜掕ˉPWM锛岄┍鍔ㄨ秴澹版尝鎺㈠ご)
    Ultrasonic_Timer_Init();           // 鍒濆鍖朤IM5寰绾ч珮绮惧害璁℃椂鍣�
    Ultrasonic_Echo_Init();            // 鍒濆鍖栧洖娉㈡帴鏀跺閮ㄤ腑鏂� (PC0鍙岃竟娌胯Е鍙�)
    Calibration_Load();                // 浠嶧lash璇诲彇鍘嗗彶鏍″噯鏁版嵁鍒癛AM

    Ultrasonic_ApplyGain(PGA112_DEFAULT_GAIN_CODE); // 璁剧疆PGA榛樿澧炵泭妗ｄ綅
    g_gain_settle_discard = 0;                     // 娓呴櫎澧炵泭绋冲畾涓㈠純鏍囧織
}

/************************* 涓荤晫闈㈡樉绀哄嚱鏁� *************************/
/**
 * @brief 缁樺埗涓昏彍鍗曞浐瀹歎I锛氭爣棰樸€佸垎鍓茬嚎銆佺増鏈€佽彍鍗曢€夐」
 */
static void Disp_Main(void)
{
    uint8_t count;

    // 椤堕儴鏍囬灞呬腑鏄剧ず
    OS_String_Show(272, 16, 32, 1, TITLE_STR);

    // 缁樺埗妯珫鐧借壊鍒嗗壊绾匡紝鍒掑垎鐣岄潰鍖哄煙 (椤堕儴绾裤€佸簳閮ㄧ嚎銆佸乏鍙冲垎闅旂嚎)
    LCD_Appoint_Clear(0, 64, 800, 72, White);    // 椤堕儴妯悜鍒嗗壊绾�
    LCD_Appoint_Clear(0, 440, 800, 448, White);  // 搴曢儴妯悜鍒嗗壊绾�
    LCD_Appoint_Clear(250, 72, 252, 440, White); // 宸﹀彸鍖哄煙绾靛悜鍒嗗壊绾�

    // 搴曢儴鐘舵€佹爮锛氱‖浠跺瀷鍙� + 杞欢鐗堟湰
    OS_String_Show(32, 456, 16, 1, MODEL_VER_STR); 
    OS_String_Show(680, 456, 16, 1, USER_VER_STR); 

    // 鍒濆鍖栬彍鍗曢€夐」鍓嶇紑涓� "-" (鏈€変腑鏍囪)
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    // 缁樺埗宸︿晶4涓彍鍗曟枃瀛�
    OS_String_Show(60, 96, 32, 1, MENU1_CHOICE1);
    OS_String_Show(60, 160, 32, 1, MENU1_CHOICE2);
    OS_String_Show(60, 224, 32, 1, MENU1_CHOICE3);
    OS_String_Show(60, 288, 32, 1, MENU1_CHOICE4);
}

/**
 * @brief 鑿滃崟鍒囨崲閫昏緫锛氭洿鏂伴€変腑绠ご銆佸埛鏂扮姸鎬佹満銆佹竻闄ゆ寜閿簨浠�
 * @param menu_sign 鐩爣鑿滃崟缂栧彿 (1~4)
 */
static void Change_Menu(uint8_t menu_sign)
{
    uint8_t count;

    // 鍏堟竻绌烘墍鏈夎彍鍗曢」鍓嶉潰鐨勯€変腑鏍囪
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    // 缁樺埗鏂伴€変腑鑿滃崟绠ご ">"锛屽苟鏇存柊鐘舵€佹満鍙橀噺
    if(menu_sign >= 1 && menu_sign <= MENU_CHOICE_NUM)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * menu_sign), 32, 1, ">");
        g_menu_sign = menu_sign;
    }
    else
    {
        g_menu_sign = 0;  // 闈炴硶鎸夐敭锛屽垏鍥炰富鑿滃崟
        Clear_Work_Area();// 娓呯┖鍙充晶宸ヤ綔鍖�
    }

    Ps2KeyValue = KeyValue_Null;  // 娑堣垂鏈鎸夐敭锛岄槻姝㈤噸澶嶈Е鍙�
}

/************************* UI宸ュ叿鍑芥暟 *************************/
/**
 * @brief 娓呯┖鍙充晶宸ヤ綔鍖哄煙 (璋冪敤鍏蜂綋娓呮枃鏈嚱鏁�)
 */
static void Clear_Work_Area(void) { Clear_Work_Text(); }

/**
 * @brief 鐢ㄧ┖鐧藉瓧绗︿覆灞€閮ㄦ竻灞忥紝鏁堢巼楂樹簬鍏ㄥ眬LCD_Clear锛岄伩鍏嶉棯鐑�
 * @note 鏍规嵁鐣岄潰甯冨眬锛屽垎鍒竻绌烘爣棰樺尯銆�9琛屼俊鎭尯銆佸簳閮ㄤ袱琛屾彁绀哄尯
 */
static void Clear_Work_Text(void)
{
    uint8_t line;
    // 鏍囬鍖烘竻灞� (32鍙峰瓧浣擄紝涓€琛�)
    OS_String_Show(280, 88, 32, 1, UI_BLANK_TEXT_32);
    // 涓棿澶氳鏂囨湰鍖烘竻灞� (24鍙峰瓧浣擄紝鍏�9琛�)
    for(line = 0; line < 9U; line++) 
    { 
        OS_String_Show(280, (uint16_t)(150 + line * 30), 24, 1, UI_BLANK_TEXT_24); 
    }
    // 搴曢儴鎻愮ず琛屾竻灞� (16鍙峰瓧浣擄紝涓よ)
    OS_String_Show(280, 400, 16, 1, UI_BLANK_TEXT_16);
    OS_String_Show(280, 420, 16, 1, UI_BLANK_TEXT_16);
}

/**
 * @brief 缁樺埗鍙充晶宸ヤ綔鍖哄ぇ鏍囬
 * @param title 鏍囬瀛楃涓�
 */
static void Draw_Work_Title(char *title) { OS_String_Show(280, 88, 32, 1, title); }

/**
 * @brief 缁樺埗搴曢儴鎸夐敭鎿嶄綔鎻愮ず (涓よ)
 * @param tip1 鎻愮ず鏂囧瓧1 (閫氬父宸︿晶鎸夐敭鍔熻兘)
 * @param tip2 鎻愮ず鏂囧瓧2 (閫氬父鍙充晶鎸夐敭鍔熻兘)
 */
static void Draw_Key_Tips(char *tip1, char *tip2) 
{ 
    OS_String_Show(280, 400, 16, 1, tip1); 
    OS_String_Show(280, 420, 16, 1, tip2); 
}

/**
 * @brief 鎸夎缁樺埗绾枃鏈� (鏍囩鍖�)
 * @param line 琛屽彿 (0璧峰)
 * @param text 鏄剧ず鏂囨湰
 */
static void Show_Text_Line(uint16_t line, char *text) 
{ 
    uint16_t y = (uint16_t)(150 + line * 30); 
    OS_String_Show(280, y, 24, 1, text); 
}

/**
 * @brief 缁樺埗"鏍囩: 鏁板€�"缁勫悎琛�
 * @param line 琛屽彿
 * @param label 鏍囩鏂囧瓧
 * @param value 寰呮樉绀烘暟鍊�
 * @param format 鏍煎紡鍖栧瓧绗︿覆 (濡� "%06.1f")
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
 * @brief 浠呯粯鍒跺彸渚ф暟鍊煎尯鍩� (涓嶅甫鏍囩)
 * @param line 琛屽彿
 * @param value 鏁板€�
 * @param format 鏍煎紡鍖栦覆
 */
static void Show_Value_Only(uint16_t line, double value, char *format) 
{ 
    char temp[24]; 
    uint16_t y = (uint16_t)(150 + line * 30); 
    sprintf(temp, format, value); 
    OS_String_Show(500, y, 24, 1, temp); 
}

/**
 * @brief 鍙充晶鏁板€煎尯鏄剧ず绾枃鏈� (瀛楃涓�)
 * @param line 琛屽彿
 * @param text 鏂囨湰
 */
static void Show_Text_Value_Only(uint16_t line, char *text) 
{ 
    uint16_t y = (uint16_t)(150 + line * 30); 
    OS_String_Show(500, y, 24, 1, text); 
}

/**
 * @brief 绛夊緟鎸夐敭鏉惧紑锛岀畝鍗曟寜閿秷鎶�+闃叉闀挎寜杩炲彂
 * @param key_value 闇€瑕佺瓑寰呴噴鏀剧殑鎸夐敭鍊� (濡侹eyValue_Enter)
 */
static void Wait_Ps2KeyRelease(uint8_t key_value)
{
    do
    {
        Ps2KeyValue = KeyValue_Null; // 娓呯┖鎸夐敭鍊�
        delay_ms(30);                // 绛夊緟30ms
    } while(Ps2KeyValue == key_value); // 鐩村埌鎸夐敭琚噴鏀�
}

/************************* 瓒呭０娉㈢‖浠堕┍鍔ㄦ牳蹇冨眰 *************************/
/**
 * @brief TIM5瀹氭椂鍣ㄥ垵濮嬪寲锛氬井绉掔骇楂樼簿搴﹁鏃�
 * @note 閫夌敤TIM5鍘熷洜锛歋TM32F4 32浣嶉€氱敤瀹氭椂鍣紝璁℃暟鑼冨洿0~4294967295锛岃繙璺濈娴嬭窛涓嶄細婧㈠嚭
 *       APB1鏃堕挓 = 84MHz锛岄鍒嗛84-1锛屽畾鏃跺櫒璁℃暟棰戠巼 = 1MHz 鈫� 1璁℃暟鍊� = 1渭s
 */
static void Ultrasonic_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  // 浣胯兘TIM5鏃堕挓
    TIM_TimeBaseStructInit(&tim_base);

    tim_base.TIM_Prescaler = 84 - 1;                      // 棰勫垎棰� 84鍒嗛 鈫� 1MHz
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;       // 鍚戜笂璁℃暟
    tim_base.TIM_Period = 0xFFFFFFFFU;                   // 鏈€澶ц嚜鍔ㄩ噸杞藉€硷紝瓒呴暱璁℃椂涓嶆孩鍑�
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;            // 涓嶅垎棰�

    TIM_TimeBaseInit(TIM5, &tim_base);
    TIM_Cmd(TIM5, ENABLE);  // 瀹氭椂鍣ㄦ寔缁紑鍚紝鍏ㄧ▼浣滀负寰璁℃椂鍣�
}

/**
 * @brief 鍥炴尝寮曡剼 + 澶栭儴涓柇鍒濆鍖�
 * @note 寮曡剼锛歅C0 娴┖杈撳叆
 *       涓柇锛欵XTI0 鍙岃竟娌胯Е鍙�(涓婂崌娌�+涓嬮檷娌�)锛岀敤浜庢崟鑾峰畬鏁村洖娉㈣剦鍐插搴�
 *       鍙岃竟娌胯Е鍙戝師鍥狅細闇€瑕佸悓鏃惰幏寰椾笂鍗囨部鍜屼笅闄嶆部鏃堕棿锛屾墠鑳借绠楄剦鍐蹭腑蹇�(娉㈠嘲)
 */
static void Ultrasonic_Echo_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);    // 浣胯兘GPIOC鏃堕挓
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  // 浣胯兘SYSCFG(澶栭儴涓柇鏄犲皠)

    // PC0 閰嶇疆涓烘诞绌鸿緭鍏� (鐢卞閮ㄨ秴澹版尝妯″潡椹卞姩)
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio_init);

    // 灏哖C0寮曡剼鏄犲皠鍒癊XTI0涓柇绾�
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

    // 澶栭儴涓柇閰嶇疆锛氬弻杈规部瑙﹀彂
    EXTI_StructInit(&exti_init);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // 涓婂崌娌�+涓嬮檷娌块兘瑙﹀彂
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
    EXTI_ClearITPendingBit(EXTI_Line0); // 娓呴櫎鍒濆涓柇鏍囧織

    // NVIC涓柇浼樺厛绾ч厤缃細浼樺厛绾ц緝楂橈紝閬垮厤涓柇寤惰繜寮曞叆寰绾ц宸�
    nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
    nvic_init.NVIC_IRQChannelSubPriority = 1;        
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

/**
 * @brief 璁剧疆PGA112绋嬫帶鏀惧ぇ鍣ㄥ鐩婃。浣�
 * @param gain_code 澧炵泭缂栫爜(0~7)锛屽搴� 1,2,4,8,16,32,64,128鍊�
 * @note 鍒囨崲澧炵泭鍚庤缃� g_gain_settle_discard 鏍囧織锛屼涪寮冮娆℃祴閲忓€肩瓑寰呯數璺ǔ瀹�
 */
static void Ultrasonic_ApplyGain(uint8_t gain_code)
{
    gain_code &= 0x07U;  // 鎺╃爜闄愬埗鑼冨洿锛岄槻姝㈤潪娉曠紪鐮�
    if(gain_code != g_ultrasonic_gain_code)
    {
        PGA112_SetGainCode(gain_code);               // 纭欢璁剧疆澧炵泭 (閫氳繃SPI)
        g_ultrasonic_gain_code = gain_code;
        g_gain_settle_discard = 1;                   // 鏍囪锛氬鐩婂垏鎹㈠悗鐢佃矾闇囪崱锛屼涪寮冮娆℃祴閲忓€�
    }
}

/**
 * @brief 鏍规嵁鍘嗗彶鍥炴尝鏃堕棿(璺濈)鑷€傚簲閫夋嫨澧炵泭妗ｄ綅
 * @note 鍘熺悊锛氳秴澹版尝杩滆窛绂讳俊鍙疯“鍑忎弗閲嶏紝璺濈瓒婅繙锛屾墍闇€鏀惧ぇ鍊嶆暟瓒婂ぇ
 *       渚濇嵁缁忛獙闃堝€煎垝鍒�8妗ｏ紝淇濊瘉鍥炴尝淇″彿骞呭害閫備腑
 * @param echo_us 鍘嗗彶鍥炴尝鏃堕棿 (渭s)
 * @return PGA澧炵泭缂栫爜 (0~7)
 */
static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us)
{
    if(echo_us < 180U)       return PGA112_GAIN_1;    // 鏋佽繎锛氫笉鏀惧ぇ
    if(echo_us < 360U)       return PGA112_GAIN_2;    
    if(echo_us < 800U)       return PGA112_GAIN_4;    
    if(echo_us < 1800U)      return PGA112_GAIN_8;    
    if(echo_us < 3600U)      return PGA112_GAIN_16;   // 涓繙璺濈寮€濮嬫斁澶�
    if(echo_us < 6500U)      return PGA112_GAIN_32;   
    if(echo_us < 9500U)      return PGA112_GAIN_64;   
    return PGA112_GAIN_128;                           // 鏋侀檺璺濈锛氭渶澶�128鍊嶆斁澶�
}

/**
 * @brief 璁＄畻鍥炴尝鑴夊啿涓績鐐�(娉㈠嘲)鏃堕棿
 * @note 鍘熺悊锛氳秴澹版尝鍥炴尝缁忔娉㈠悗杈撳嚭涓烘柟娉紝淇″彿鑳介噺宄板€煎嚭鐜板湪鏂规尝涓績
 *       浣跨敤涓績鏃跺埢浠ｆ浛杈规部锛屽彲浠ユ彁楂樻祴璺濈簿搴︼紝鍑忓皬鑴夊甯︽潵鐨勮宸�
 * @param rise_us 涓婂崌娌挎椂闂� (渭s)
 * @param fall_us 涓嬮檷娌挎椂闂� (渭s)
 * @return 娉㈠嘲瀵瑰簲寰鏃堕棿
 */
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us)
{
    uint32_t width;
    // 寮傚父淇濇姢锛氫笅闄嶆部灏忎簬涓婂崌娌�(璁℃暟鍣ㄦ孩鍑�/骞叉壈)锛岀洿鎺ヨ繑鍥炰笂鍗囨部
    if(fall_us <= rise_us)
    {
        return rise_us;
    }
    width = fall_us - rise_us;
    return rise_us + width / 2U; // 鍙栬剦鍐蹭腑蹇�
}

/**
 * @brief 鑷€傚簲澧炵泭鍑嗗锛氭祴閲忓け璐ユ椂閫愮骇鎶崌澧炵泭
 * @param retry_count 澶辫触閲嶈瘯娆℃暟 (0~3)
 * @note 鍩轰簬涓婁竴娆℃湁鏁堝洖娉㈡椂闂撮€夋嫨鍩虹澧炵泭锛岃嫢杩炵画澶辫触鍒欓澶栧鍔犲鐩婃。浣�
 */
static void Ultrasonic_PrepareGain(uint8_t retry_count)
{
    // 鍩轰簬涓婁竴娆℃湁鏁堣窛绂婚€夋嫨鍩虹澧炵泭
    uint8_t gain_code = Ultrasonic_SelectGainCode(g_last_echo_us);

    // 杩炵画娴嬮噺澶辫触锛岄€愮骇澧炲姞澧炵泭 (姣忔澶辫触鎻愬崌涓€妗�)
    if(retry_count > ULTRASONIC_GAIN_RETRY_MAX)
    {
        retry_count = ULTRASONIC_GAIN_RETRY_MAX;
    }
    gain_code = (uint8_t)(gain_code + retry_count);

    if(gain_code > PGA112_GAIN_128)
    {
        gain_code = PGA112_GAIN_128; // 闄愬埗鏈€澶у鐩�
    }
    Ultrasonic_ApplyGain(gain_code);
    delay_us(20); // 绛夊緟SPI閰嶇疆+杩愭斁鐢佃矾绋冲畾 (绾�20渭s)
}

/**
 * @brief 璁剧疆鍥炴尝鏈夋晥鎺ユ敹鏃堕棿绐楀彛锛岃繃婊ょ獥鍙ｅ骞叉壈淇″彿
 * @param min_us 绐楀彛涓嬮檺 (渭s)
 * @param max_us 绐楀彛涓婇檺 (渭s)
 * @note 鑻ュ弬鏁版棤鏁堝垯鎭㈠鍏ㄨ寖鍥寸獥鍙�
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
 * @brief 鍔ㄦ€佽缃窡韪獥鍙ｏ細閿佸畾鐩爣鍚庣缉灏忕獥鍙ｃ€佷涪澶辩洰鏍囧悗鍏ㄥ煙鎼滅储
 * @note 璺熻釜绐楀彛鍒╃敤鐩爣绌洪棿鐩稿叧鎬э紝鍦ㄥ巻鍙插€奸檮杩戝紑绐勭獥鍙ｏ紝鎻愰珮鎶楀共鎵拌兘鍔�
 *       鑻ヨ繛缁涪澶辩洰鏍囧垯鎵╁ぇ绐楀彛閲嶆柊鎹曡幏
 */
static void Ultrasonic_SetTrackingWindow(void)
{
    const uint32_t margin_us = ULTRASONIC_TRACK_MARGIN_US;
    if(g_reacquire_ignore_near != 0U)
    {
        // 閲嶆悳绱㈡ā寮忥細灞忚斀杩戣窛绂� (閬垮紑鐩插尯)锛屼粠鏈€灏忔湁鏁堣繙璺濈寮€濮嬫悳绱�
        Ultrasonic_SetAcceptWindow(ULTRASONIC_REACQUIRE_MIN_US, ULTRASONIC_TIMEOUT_US);
    }
    else if(g_tracking_valid == 0U)
    {
        // 鏈攣瀹氱洰鏍囷細鍏ㄥ紑绐楀彛锛屽叏鍩熸悳绱�
        Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);
    }
    else if(g_last_echo_us > margin_us)
    {
        // 宸查攣瀹氫笖鍘嗗彶鍊煎ぇ浜庝綑閲忥細浠ュ巻鍙插€间负涓績锛屽乏鍙虫墿灞曚綑閲忥紝绐勭獥鍙ｈ窡韪�
        Ultrasonic_SetAcceptWindow(g_last_echo_us - margin_us, g_last_echo_us + margin_us);
    }
    else
    {
        // 杩戣窛绂荤洰鏍囷細涓嬮檺璁句负鐩插尯 (閬垮厤鑷縺)锛屼笂闄愪负鍘嗗彶鍊�+浣欓噺
        Ultrasonic_SetAcceptWindow(ULTRASONIC_BLANKING_US, g_last_echo_us + margin_us);
    }
}

/**
 * @brief 鍗曟瓒呭０娉㈠彂灏�+鍥炴尝鎹曡幏 (鐗╃悊灞�)
 * @param echo_us 杈撳嚭锛氭湰娆″洖娉㈡椂闂� (渭s)
 * @return 1=鎹曡幏鎴愬姛 0=瓒呮椂澶辫触
 * @note 闃诲杞绛夊緟鍥炴尝锛岃秴鏃舵椂闂� ULTRASONIC_TIMEOUT_US
 */
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us)
{
    uint32_t timeout;
    // 1. 澶嶄綅鎵€鏈変腑鏂姸鎬佹爣蹇�
    g_echo_captured = 0;
    g_measure_active = 1;
    g_echo_time_us = 0;
    g_echo_rise_us = 0;
    g_echo_fall_us = 0;
    g_echo_rise_seen = 0;

    // 2. 娓呯┖瀹氭椂鍣ㄨ鏁板€� + 娓呴櫎涓柇鎸傝捣 (纭繚浠�0寮€濮嬭鏃�)
    TIM_SetCounter(TIM5, 0);
    EXTI_ClearITPendingBit(EXTI_Line0);

    // 3. 椹卞姩鎺㈠ご鍙戝皠涓€缁�40kHz瓒呭０娉㈣剦鍐蹭覆 (閫氬父8涓剦鍐�)
    Ultrasonic_FireBurst();

    // 4. 闃诲杞绛夊緟鍥炴尝锛岃秴鏃跺垯閫€鍑�
    for(timeout = 0; timeout < ULTRASONIC_TIMEOUT_US / 10U; timeout++)
    {
        if(g_echo_captured != 0U)
        {
            *echo_us = g_echo_time_us;
            g_measure_active = 0; // 鍏抽棴娴嬮噺绐楀彛
            return 1;
        }
        delay_us(10);
    }

    // 瓒呮椂澶辫触
    g_measure_active = 0;
    return 0;
}

/**
 * @brief 甯﹀绾ф护娉㈢殑鎵归噺娴嬮噺锛氶噰鏍�+澧炵泭鑷€傚簲+鑱氱被闄嶅櫔
 * @param echo_us 杈撳嚭鏈€缁堟湁鏁堝洖娉㈡椂闂� (渭s)
 * @return 1=閲囨牱鎴愬姛 0=鍏ㄩ儴澶辫触
 * @note 绠楁硶娴佺▼锛�
 *       1. 鑷€傚簲澧炵泭璋冩暣
 *       2. 杩炵画閲囬泦 ULTRASONIC_FILTER_SAMPLES 涓湁鏁堟牱鏈�
 *       3. 鎻掑叆鎺掑簭
 *       4. 瀵绘壘鏈€瀵嗛泦鏍锋湰绨� (瀹瑰繊 卤90渭s 娉㈠姩)
 *       5. 鍙栫皣涓綅鏁颁綔涓烘渶缁堢粨鏋�
 */
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us)
{
    uint32_t samples[ULTRASONIC_FILTER_SAMPLES]; // 閲囨牱缂撳啿鍖�
    uint8_t valid_count = 0;                     // 鏈夋晥鏍锋湰璁℃暟
    uint8_t attempts = 0;                        // 鎬诲彂灏勬鏁� (闃叉鏃犻檺寰幆)
    uint8_t gain_retry = 0;                      // 澧炵泭閲嶈瘯璁℃暟
    uint8_t miss_count = 0;                      // 杩炵画涓㈠け鍥炴尝璁℃暟
    uint8_t index;
    uint32_t reacquire_min_us = ULTRASONIC_REACQUIRE_MIN_US;

    // 鏍″噯鏁版嵁鏈夋晥鏃讹紝浣跨敤绗竴鏍″噯鐐规椂闂翠綔涓洪噸鎼滅储涓嬮檺 (鏇寸簿纭�)
    if((g_calib_valid != 0U) && (g_calib.point_us[0] > reacquire_min_us))
    {
        reacquire_min_us = g_calib.point_us[0];
    }

    // 寰幆閲囬泦锛屽噾澶熸寚瀹氭暟閲忔湁鏁堟牱鏈紝鏈€澶氬厑璁搁澶�20娆″彂灏勬満浼�
    while(attempts < (ULTRASONIC_FILTER_SAMPLES + 20U) && valid_count < ULTRASONIC_FILTER_SAMPLES)
    {
        uint32_t sample = 0;
        Ultrasonic_PrepareGain(gain_retry); // 鍔ㄦ€侀厤缃鐩� (鏍规嵁涓婃缁撴灉鍜屽け璐ユ鏁�)
        attempts++;

        if(Ultrasonic_MeasureOnce(&sample) != 0U)
        {
            // 澧炵泭鍒氬垏鎹紝鐢佃矾鏈ǔ瀹氾紝涓㈠純鑴忔暟鎹�
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
                delay_ms(8);
                continue;
            }
            // 閲嶆悳绱㈡ā寮忎笅灞忚斀杩戣窛绂诲共鎵� (閬垮紑鐩插尯)
            if((g_reacquire_ignore_near != 0U) && (sample < reacquire_min_us))
            {
                continue;
            }
            // 瀛樺叆鏈夋晥鏍锋湰
            samples[valid_count++] = sample;
            g_last_echo_us = sample;         // 鏇存柊鍘嗗彶鍊�
            gain_retry = 0;                  // 鎴愬姛鍒欐竻闄ら噸璇曡鏁�
            miss_count = 0;
            g_reacquire_ignore_near = 0U;    // 鎴愬姛鎹曡幏鍚庨€€鍑洪噸鎼滅储妯″紡
        }
        else
        {
            // 鍗曟娴嬮噺澶辫触澶勭悊
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
            }
            else
            {
                // 鏈秴闄愬垯鎻愬崌澧炵泭閲嶈瘯 (閫愮骇澧炲姞)
                if(gain_retry < ULTRASONIC_GAIN_RETRY_MAX)
                {
                    gain_retry++;
                }
                // 杩炵画涓㈠け鍥炴尝锛屽垽瀹氱洰鏍囦涪澶憋紝杩涘叆鍏ㄥ煙閲嶆悳绱�
                if(g_tracking_valid != 0U)
                {
                    miss_count++;
                    if(miss_count >= ULTRASONIC_REACQUIRE_MISSES)
                    {
                        // 閲嶇疆鎵€鏈夌姸鎬侊紝鎵╁ぇ鎼滅储绐楀彛
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
        delay_ms(8); // 闄嶄綆鍙戝皠鍗犵┖姣旓紝闃叉澹版尝鍙犲姞褰㈡垚椹绘尝骞叉壈
    }

    if(valid_count == 0U)
    {
        return 0; // 鏃犳湁鏁堟牱鏈紝娴嬮噺澶辫触
    }

    // 鏍锋湰鎺掑簭 (鍗囧簭)
    Sort_Samples(samples, valid_count);

    // 鑱氱被绠楁硶锛氶€夊彇鏈€瀵嗛泦鏍锋湰绨囩殑涓棿鍊间綔涓烘渶缁堢粨鏋滐紝鍓旈櫎璺冲彉骞叉壈
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
        best_mid = (uint8_t)(best_start + best_count / 2U);
        *echo_us = samples[best_mid];
    }
    else
    {
        // 鏍锋湰杈冨皯锛岀洿鎺ュ彇涓棿鍊�
        *echo_us = samples[valid_count / 2U];
    }

    g_last_echo_us = *echo_us;
    return 1;
}

/**
 * @brief 鎻掑叆鎺掑簭锛氬閲囨牱鏁扮粍鍗囧簭鎺掑垪锛岀敤浜庡悗缁仛绫汇€佸幓鏋佸€�
 * @param data 寰呮帓搴忔暟缁�
 * @param length 鏁扮粍闀垮害
 */
static void Sort_Samples(uint32_t *data, uint8_t length)
{
    uint8_t i;
    for(i = 1U; i < length; i++)
    {
        uint32_t key = data[i];
        int8_t j = (int8_t)i - 1;
        // 鍚戝墠绉讳綅锛屾壘鍒版彃鍏ヤ綅缃�
        while(j >= 0 && data[j] > key)
        {
            data[j + 1] = data[j];
            j--;
        }
        data[j + 1] = key;
    }
}

/************************* 鏍″噯涓庤窛绂昏浆鎹㈡暟鎹眰 *************************/
/**
 * @brief 浠嶧lash鎸囧畾鍦板潃璇诲彇鏍″噯鏁版嵁鍒癛AM
 * @note 鐩存帴鎸囬拡寮哄埗杞崲璇诲彇锛屼笉缁忚繃缂撳瓨锛岃鍙栧悗鑷姩鏍￠獙鍚堟硶鎬�
 */
static void Calibration_Load(void)
{
    // 鍦板潃寮哄埗杞负缁撴瀯浣撴寚閽堬紝鐩存帴璇诲彇Flash鏁版嵁
    const UltrasonicCalibData *stored = (const UltrasonicCalibData *)ULTRASONIC_FLASH_ADDR;
    g_calib = *stored; 
    g_calib_valid = Calibration_IsValid(&g_calib); // 鏍￠獙鏁版嵁鍚堟硶鎬�
}

/**
 * @brief 鏍￠獙鏍″噯鏁版嵁鏄惁鍚堟硶锛氶瓟鏁般€佺増鏈€佹椂搴忛€昏緫鏍￠獙
 * @param calib 寰呮牎楠岀粨鏋勪綋
 * @return 1=鍚堟硶 0=闈炴硶
 */
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib)
{
    uint8_t index;
    // 1. 榄旀暟+鐗堟湰鏍￠獙 (闃叉璇诲彇鍒伴殢鏈篎lash鍐呭)
    if(calib->magic != ULTRASONIC_FLASH_MAGIC || calib->version != ULTRASONIC_FLASH_VERSION)
    {
        return 0;
    }
    // 2. 鐗╃悊閫昏緫鏍￠獙锛氳窛绂昏秺杩滐紝鍥炴尝鏃堕棿蹇呴』鍗曡皟閫掑锛屼笖涓嶈秴杩囨渶澶ч噺绋�
    for(index = 0; index < 5U; index++)
    {
        if(calib->point_us[index] == 0U || calib->point_us[index] > ULTRASONIC_TIMEOUT_US)
        {
            return 0; // 鏃堕棿瓒呭嚭鍚堟硶鑼冨洿
        }
        if(index > 0U && calib->point_us[index] <= calib->point_us[index - 1U])
        {
            return 0; // 闈炲崟璋冮€掑 (鐗╃悊涓婁笉鍙兘)
        }
    }
    return 1;  
}

/**
 * @brief 灏嗘牎鍑嗘暟鎹啓鍏lash Sector11
 * @param calib 寰呭啓鍏ユ牎鍑嗙粨鏋勪綋
 * @return 1=鍐欏叆鎴愬姛 0=澶辫触
 * @note Flash鐗规€э細鍙兘鎿﹂櫎(鍐欏叆1)鍜岀紪绋�(鍐欏叆0)锛屼笉鑳藉崟鐙敼鍐欏瓧鑺傦紱鎿﹂櫎鎸夋墖鍖烘搷浣�
 *       Sector11澶у皬涓�128KB锛屽湴鍧€鑼冨洿 0x080E0000 - 0x080FFFFF
 */
static uint8_t Calibration_Save(const UltrasonicCalibData *calib)
{
    FLASH_Status status = FLASH_COMPLETE;
    const uint32_t *words = (const uint32_t *)calib; 
    uint32_t address = ULTRASONIC_FLASH_ADDR;
    uint32_t index;

    FLASH_Unlock(); // 瑙ｉ攣Flash鍐欎繚鎶�
    // 娓呴櫎Flash閿欒鏍囧織 (涓轰笂涓€姝ュ彲鑳界殑閿欒娓呯悊)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 鏁存墖鍖烘摝闄� (鎵€鏈変綅鍙樹负1)
    status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    if(status == FLASH_COMPLETE)
    {
        // 鎸�32bit瀛楅€愬瓧鍐欏叆 (缁撴瀯浣撳ぇ灏�32瀛楄妭锛屽叡8涓瓧)
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
    FLASH_Lock(); // 閲嶆柊涓婇攣淇濇姢Flash

    return (uint8_t)(status == FLASH_COMPLETE); 
}

/**
 * @brief 鏍″噯鏃舵牴鎹爣鍑嗚窛绂昏缃笓灞炴帴鏀剁獥鍙ｏ紝鎻愰珮鏍囧畾绮惧害
 * @param distance_mm 褰撳墠鏍囧畾鏍囧噯璺濈 (mm)
 * @note 閫氳繃闄愬埗绐楀彛鑼冨洿锛岄伩鍏嶆牎鍑嗘椂閲囬泦鍒板寰勫共鎵版垨閭昏繎鐗╀綋鍥炴尝
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
 * @brief 鏃犳牎鍑嗘暟鎹椂锛屼娇鐢ㄧ悊鎯崇墿鐞嗗叕寮忚绠楄窛绂� (鍥為€€鏂规)
 * @param echo_us 鍥炴尝鏃堕棿(渭s)
 * @return 璁＄畻璺濈(mm)
 * @note 鐞嗚鍏紡锛氳窛绂�(mm) = 鏃堕棿(渭s) * 0.1715
 *       瀹為檯浣跨敤 0.164866 绯绘暟鏄负浜嗛€傞厤HC-SR04妯″潡鐨勫父瑙佸亸宸�
 */
static float Convert_Time_To_Distance_Default(uint32_t echo_us)
{
    float distance = (float)echo_us * 0.164866f;
    // 闄愬箙鍒伴噺绋嬭寖鍥� 10mm ~ 1300mm
    if(distance < 10.0f) distance = 10.0f;
    if(distance > 1300.0f) distance = 1300.0f;
    return distance;
}

/**
 * @brief 鍒嗘绾挎€ф彃鍊艰窛绂绘崲绠� (浣跨敤鏍″噯鏁版嵁锛屼慨姝ｇ郴缁熻宸�)
 * @param echo_us 鍥炴尝鏃堕棿(渭s)
 * @return 淇鍚庤窛绂�(mm)
 * @note 鍘熺悊锛氬皢鍏ㄩ噺绋嬪垎涓哄娈电洿绾匡紝鐢ㄦ爣瀹氱偣鎷熷悎闈炵嚎鎬ц宸�
 *       渚嬪锛氫娇鐢�5涓爣瀹氱偣锛屽垎鎴�4娈电洿绾匡紝涓ょ偣寮忔彃鍊�
 */
static float Convert_Time_To_Distance(uint32_t echo_us)
{
    uint8_t index;
    float x0, x1;  // 妯潗鏍囷細瀹為檯娴嬮噺鍥炴尝鏃堕棿 (渭s)
    float y0, y1;  // 绾靛潗鏍囷細鏍囧噯鐗╃悊璺濈 (mm)
    float distance;

    if(g_calib_valid == 0U)
    {
        return Convert_Time_To_Distance_Default(echo_us); // 鏃犳牎鍑嗗垯鐢ㄧ悊鎯冲叕寮�
    }

    // 鏍规嵁鍥炴尝鏃堕棿鍒ゆ柇钀藉湪鍝竴娈垫姌绾垮尯闂村唴
    index = 0U;
    while((index < 3U) && (echo_us > g_calib.point_us[index + 1U]))
    {
        index++;
    }

    x0 = (float)g_calib.point_us[index];
    x1 = (float)g_calib.point_us[index + 1U];
    y0 = (float)k_calib_distance_mm[index];
    y1 = (float)k_calib_distance_mm[index + 1U];

    if(x1 <= x0) // 闃查櫎闆朵繚鎶�
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    // 涓ょ偣寮忕洿绾挎彃鍊�: y = y0 + (x - x0)*(y1 - y0)/(x1 - x0)
    distance = y0 + ((float)echo_us - x0) * (y1 - y0) / (x1 - x0);

    // 闄愬箙鍒版湁鏁堥噺绋�
    if(distance < (float)k_calib_distance_mm[0]) distance = (float)k_calib_distance_mm[0];
    if(distance > 1300.0f) distance = 1300.0f;
    return distance;
}

/************************* 鑿滃崟閫昏緫浜や簰灞� *************************/
/**
 * @brief 鎵嬪姩娴嬮噺鐣岄潰涓氬姟閫昏緫
 * @note 鎸変竴娆nter瑙﹀彂涓€娆℃护娉㈡祴璺濓紝鏄剧ず鏃堕棿銆佽窛绂汇€佹牎鍑嗙姸鎬併€佸鐩婄瓑
 *       鎸夎繑鍥為敭閫€鍑哄埌涓昏彍鍗�
 */
static void MenuHandler_Measure(void)
{
    char value_text[24];
    Draw_Work_Title("鎵嬪姩娴嬮噺");
    Draw_Key_Tips("Enter: 娴嬮噺涓€娆�", "Back: 閫€鍑�");

    g_tracking_valid = 0;
    g_reacquire_ignore_near = 0U;

    // 鍥哄畾鏂囨湰缁樺埗 (鏍囩)
    OS_String_Show(280, 150, 24, 1, "娴嬮噺鏃堕棿(us)");
    OS_String_Show(280, 180, 24, 1, "娴嬮噺璺濈(mm)");
    OS_String_Show(280, 210, 24, 1, "榛樿璺濈(mm)");
    OS_String_Show(280, 240, 24, 1, "鏍″噯鐘舵€�");
    OS_String_Show(280, 270, 24, 1, "鍓嶇澧炵泭(x)");
    OS_String_Show(280, 300, 24, 1, "鎻愮ず淇℃伅");

    Show_Text_Value_Only(0, "-----");
    Show_Text_Value_Only(1, "------");
    Show_Text_Value_Only(2, "------");
    Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "宸叉牎鍑�" : "鏈牎鍑�");
    Show_Text_Value_Only(4, "008");
    Show_Text_Value_Only(5, "绛夊緟Enter");

    // 鎵嬪姩娴嬮噺锛氭瘡娆℃寜Enter鍙Е鍙戜竴娆″鏍锋湰婊ゆ尝娴嬮噺锛屼繚鐣欑粨鏋滅洿鍒颁笅涓€娆¤Е鍙�
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
            Show_Text_Value_Only(5, "姝ｅ湪娴嬮噺  ");

            Ultrasonic_SetTrackingWindow(); // 鏍规嵁璺熻釜鐘舵€佽缃帴鏀剁獥鍙�

            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                float distance = Convert_Time_To_Distance(echo_us);
                g_tracking_valid = 1; // 娴嬭窛鎴愬姛锛屾爣璁拌窡韪湁鏁�

                sprintf(value_text, "%05lu", (unsigned long)echo_us);
                Show_Text_Value_Only(0, value_text);

                sprintf(value_text, "%06.1f", (double)distance);
                Show_Text_Value_Only(1, value_text);

                sprintf(value_text, "%06.1f", (double)Convert_Time_To_Distance_Default(echo_us));
                Show_Text_Value_Only(2, value_text);

                Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "宸叉牎鍑�  " : "鏈牎鍑�  ");
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(5, "娴嬮噺瀹屾垚  ");
            }
            else
            {
                // 娴嬮噺澶辫触锛氳繘鍏ラ噸鎼滅储妯″紡
                g_reacquire_ignore_near = 1U;
                g_tracking_valid = 0;
                Show_Text_Value_Only(3, "娴嬮噺澶辫触  ");
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(5, "妫€鏌ユ帰澶�  ");
            }
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }

        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0); // 杩斿洖涓昏彍鍗�
}

/**
 * @brief 瓒呭０娉㈡牎鍑嗚彍鍗曞鐞嗗嚱鏁�
 * @details 鍒嗘瀹屾垚5涓爣鍑嗚窛绂荤偣閲囨牱銆佸悎娉曟€ф牎楠屻€丗lash淇濆瓨鏍″噯鏁版嵁
 *          鏀寔鎸夐敭纭閲囨牱銆佽繑鍥為敭閫€鍑烘牎鍑嗘祦绋�
 * @note 鏍″噯姝ラ锛�
 *       1. 鍦ㄦ寚瀹氳窛绂绘斁缃弽灏勬澘 (渚濇 100,300,600,900,1300mm)
 *       2. 鎸夌‘璁ら敭杩涜鑷姩閲囨牱
 *       3. 瀹屾垚鍏ㄩ儴5鐐瑰悗鑷姩鏍￠獙骞朵繚瀛樿嚦Flash
 */
static void MenuHandler_Calibrate(void)
{
    UltrasonicCalibData new_calib;
    uint8_t step = 0;
    uint8_t last_step = 0xFFU;
    char line[24];

    // 鍒濆鍖栨牎鍑嗘暟鎹粨鏋� (鍐欏叆榄旀暟銆佺増鏈紝娓呯┖鏃堕棿鐐�)
    new_calib.magic = ULTRASONIC_FLASH_MAGIC;
    new_calib.version = ULTRASONIC_FLASH_VERSION;
    new_calib.point_us[0] = 0; new_calib.point_us[1] = 0;
    new_calib.point_us[2] = 0; new_calib.point_us[3] = 0;
    new_calib.point_us[4] = 0; new_calib.reserved[0] = 0;

    Draw_Work_Title("璺濈鏍″噯");
    Draw_Key_Tips("纭寮€濮嬫牎鍑�", "杩斿洖閫€鍑烘牎鍑�");
    OS_String_Show(280, 150, 24, 1, "鏍″噯鎻愮ず");
    OS_String_Show(280, 180, 24, 1, "褰撳墠鐘舵€�");
    OS_String_Show(280, 210, 24, 1, "100mm(us)");
    OS_String_Show(280, 240, 24, 1, "300mm(us)");
    OS_String_Show(280, 270, 24, 1, "600mm(us)");
    OS_String_Show(280, 300, 24, 1, "900mm(us)");
    OS_String_Show(280, 330, 24, 1, "1300mm(us)");
    OS_String_Show(280, 360, 24, 1, "褰撳墠娴嬪€�(us)");
    OS_String_Show(280, 390, 24, 1, "鏍″噯缁撴灉");
    Show_Text_Value_Only(1, "绛夊緟鏍″噯");
    Show_Text_Value_Only(2, "00000"); Show_Text_Value_Only(3, "00000");
    Show_Text_Value_Only(4, "00000"); Show_Text_Value_Only(5, "00000");
    Show_Text_Value_Only(6, "00000"); Show_Text_Value_Only(7, "00000");
    Show_Text_Value_Only(8, "绛夊緟鏍″噯");

    while(Ps2KeyValue != KeyValue_Back)
    {
        if(step != last_step)
        {
            sprintf(line, "瀵瑰噯%04umm", k_calib_distance_mm[step]);
            Show_Text_Value_Only(0, line);
            last_step = step;
        }

        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;

            Show_Text_Value_Only(0, "寮€濮嬫牎鍑�");
            Show_Text_Value_Only(1, "姝ｅ湪鏍″噯");
            Calibration_SetMeasureWindow(k_calib_distance_mm[step]); // 璁剧疆绐楀彛
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                new_calib.point_us[step] = echo_us;
                sprintf(line, "%05lu", (unsigned long)echo_us);
                Show_Text_Value_Only(7, line);
                Show_Text_Value_Only((uint16_t)(2 + step), line);
                Show_Text_Value_Only(1, "閲囨牱瀹屾垚");
                step++;

                if(step >= 5U) // 5涓偣鍏ㄩ儴瀹屾垚
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
                        Show_Text_Value_Only(8, "鏍″噯瀹屾垚  ");
                    }
                    else if(valid_ok == 0U)
                    {
                        Show_Text_Value_Only(1, "妫€鏌ョ偣浣�");
                        Show_Text_Value_Only(8, "POINT ERR ");
                    }
                    else
                    {
                        Show_Text_Value_Only(1, "鍐欏叆澶辫触");
                        Show_Text_Value_Only(8, "FLASH ERR ");
                    }
                    delay_ms(1000);
                    break;
                }
            }
            else
            {
                Show_Text_Value_Only(1, "鏍″噯瓒呮椂");
                Show_Text_Value_Only(8, "鏍″噯澶辫触");
            }
            Show_Text_Value_Only(8, "鏉惧紑纭閿�");
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);
    Change_Menu(0);
}

/**
 * @brief 绯荤粺鐘舵€佹煡鐪嬭彍鍗曞鐞嗗嚱鏁�
 * @details 灞曠ず鏍″噯鏈夋晥鎬с€�5涓牎鍑嗙偣鍘熷us鍊笺€佸綋鍓嶅鐩�
 *          鏀寔鎸変笅纭閿繘琛屼竴娆″疄鏃舵祴閲忓苟鏄剧ず缁撴灉
 */
static void MenuHandler_Status(void)
{
    char value_text[24];

    Draw_Work_Title("绯荤粺鐘舵€�");
    Draw_Key_Tips("纭鏌ョ湅娴嬮噺", "杩斿洖閫€鍑烘煡鐪�");
    Ultrasonic_SetAcceptWindow(0, ULTRASONIC_TIMEOUT_US);
    OS_String_Show(280, 150, 24, 1, "鏍″噯鐘舵€�");
    OS_String_Show(280, 180, 24, 1, "100mm(us)");
    OS_String_Show(280, 210, 24, 1, "300mm(us)");
    OS_String_Show(280, 240, 24, 1, "600mm(us)");
    OS_String_Show(280, 270, 24, 1, "900mm(us)");
    OS_String_Show(280, 300, 24, 1, "1300mm(us)");
    OS_String_Show(280, 330, 24, 1, "褰撳墠澧炵泭(x)");
    OS_String_Show(280, 360, 24, 1, "瀹炴椂娴嬮噺鏃堕棿(us)");
    OS_String_Show(280, 390, 24, 1, "瀹炴椂娴嬮噺璺濈(mm)");

    Show_Text_Value_Only(0, (g_calib_valid != 0U) ? "鏍″噯鏈夋晥" : "鏁版嵁鏃犳晥");
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[0]); Show_Text_Value_Only(1, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[1]); Show_Text_Value_Only(2, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[2]); Show_Text_Value_Only(3, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[3]); Show_Text_Value_Only(4, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[4]); Show_Text_Value_Only(5, value_text);
    sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code)); Show_Text_Value_Only(6, value_text);
    Show_Text_Value_Only(7, "00000");
    Show_Text_Value_Only(8, "0000.0");

    while(Ps2KeyValue != KeyValue_Back)
    {
        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                sprintf(value_text, "%05lu", (unsigned long)echo_us);
                Show_Text_Value_Only(7, value_text);
                sprintf(value_text, "%06.1f", (double)Convert_Time_To_Distance(echo_us));
                Show_Text_Value_Only(8, value_text);
            }
            else
            {
                Show_Text_Value_Only(7, "娴嬮噺澶辫触");
                Show_Text_Value_Only(8, "0000.0");
            }
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

/**
 * @brief PGA112绋嬫帶澧炵泭娴嬭瘯鑿滃崟
 * @details 寮€鍚秴澹版尝PWM鍙戝皠 (40kHz 浜掕ˉPWM)锛岄€氳繃鍔犲噺鎸夐敭鍒囨崲PGA112澧炵泭妗ｄ綅(1~128鍊�)
 *          鎸変笅杩斿洖閿叧闂璓WM骞堕€€鍑哄綋鍓嶈彍鍗曪紝鏂逛究璋冭瘯鍓嶇鐢佃矾銆�
 */
static void MenuHandler_PGA_Test(void)
{
    uint8_t gain_index = 3U;  // 榛樿澧炵泭绱㈠紩锛屽搴�8鍊嶅鐩�

    // PGA112 8妗ｅ鐩婇厤缃爜锛�1/2/4/8/16/32/64/128鍊�
    const uint8_t gain_codes[8] =
    {
        PGA112_GAIN_1, PGA112_GAIN_2, PGA112_GAIN_4, PGA112_GAIN_8,
        PGA112_GAIN_16, PGA112_GAIN_32, PGA112_GAIN_64, PGA112_GAIN_128
    };

    char value_text[24];  // 瀛楃涓茬紦瀛�

    Ultrasonic_PWM_OutputEnable();                     // 寮€鍚秴澹版尝PWM杈撳嚭
    Ultrasonic_ApplyGain(gain_codes[gain_index]);      // 鍔犺浇鍒濆澧炵泭

    // 鐣岄潰鏍囬銆佹寜閿彁绀恒€佸浐瀹氭枃鏈�
    Draw_Work_Title("绋嬫帶澧炵泭璋冭妭");
    Draw_Key_Tips("+/-璋冭妭澧炵泭", "Back閫€鍑哄苟鍏抽棴PWM");
    OS_String_Show(280, 150, 24, 1, "PWM杈撳嚭鐘舵€�");
    OS_String_Show(280, 180, 24, 1, "杈撳嚭鏂瑰紡");
    OS_String_Show(280, 210, 24, 1, "杈撳嚭棰戠巼(Hz)");
    OS_String_Show(280, 240, 24, 1, "杈撳嚭鍗犵┖姣�(%)");
    OS_String_Show(280, 270, 24, 1, "褰撳墠澧炵泭(x)");
    OS_String_Show(280, 300, 24, 1, "澧炵泭妗ｄ綅");
    OS_String_Show(280, 330, 24, 1, "娉㈠舰璇存槑");
    OS_String_Show(280, 360, 24, 1, "褰撳墠鎻愮ず");

    // 鍒濆鍖栫晫闈㈠浐瀹氬唴瀹�
    Show_Text_Value_Only(0, "寮€鍚�");
    Show_Text_Value_Only(1, "浜掕ˉPWM");
    Show_Text_Value_Only(2, "040000");
    Show_Text_Value_Only(3, "050");
    sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
    Show_Text_Value_Only(4, value_text);
    Show_Text_Value_Only(5, "1/2/4/8/16/32/64/128");
    Show_Text_Value_Only(6, "PD12/PD13杈撳嚭");
    Show_Text_Value_Only(7, "绛夊緟璋冭妭");

    // 涓诲惊鐜細鎸夐敭璋冭妭澧炵泭锛岃繑鍥為敭閫€鍑�
    while(Ps2KeyValue != KeyValue_Back)
    {
        // 澧炲姞澧炵泭
        if(Ps2KeyValue == KeyValue_Add)
        {
            Ps2KeyValue = KeyValue_Null;
            if(gain_index < 7U) // 鏈埌鏈€澶ф。浣�
            {
                gain_index++;
                Ultrasonic_ApplyGain(gain_codes[gain_index]); // 璁剧疆鏂板鐩�
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(7, "澧炵泭宸茶皟澶�");
            }
            else
            {
                Show_Text_Value_Only(7, "宸插埌鏈€澶у鐩�");
            }
        }
        // 鍑忓皬澧炵泭
        else if(Ps2KeyValue == KeyValue_Minus)
        {
            Ps2KeyValue = KeyValue_Null;
            if(gain_index > 0U) // 鏈埌鏈€灏忔。浣�
            {
                gain_index--;
                Ultrasonic_ApplyGain(gain_codes[gain_index]); // 璁剧疆鏂板鐩�
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(7, "澧炵泭宸茶皟灏�");
            }
            else
            {
                Show_Text_Value_Only(7, "宸插埌鏈€灏忓鐩�");
            }
        }
        delay_ms(20);
    }

    Ultrasonic_PWM_OutputDisable();  // 鍏抽棴PWM杈撳嚭
    Ps2KeyValue = KeyValue_Null;     // 娓呯┖鎸夐敭鐘舵€�
    Change_Menu(0);                  // 杩斿洖涓昏彍鍗�
}

/************************* 涓柇鏈嶅姟鍑芥暟 *************************/

/**
 * @brief EXTI0 澶栭儴涓柇鏈嶅姟鍑芥暟
 * @note 鍥炴尝淇″彿(PC0)杈规部瑙﹀彂涓柇锛岄噰鐢�**鐘舵€佹満**鎹曡幏瓒呭０娉㈠洖娉㈡椂搴�
 * @娴佺▼ 鐩插尯杩囨护 鈫� 鎹曡幏涓婂崌娌� 鈫� 鎹曡幏涓嬮檷娌� 鈫� 鏍￠獙鑴夊啿鏈夋晥鎬� 鈫� 鏍囪閲囨牱瀹屾垚
 * @attention 璇ヤ腑鏂紭鍏堢骇杈冮珮锛屽簲灏藉揩澶勭悊锛岄伩鍏嶉樆濉炲叾浠栦腑鏂�
 */
void EXTI0_IRQHandler(void)
{
    // 鍒ゆ柇鏄惁涓� EXTI_Line0 涓柇瑙﹀彂
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 浠呭湪娴嬮噺绐楀彛寮€鍚椂锛屾墠鍝嶅簲鍥炴尝淇″彿锛屽睆钄芥潅娉㈠共鎵�
        if(g_measure_active != 0U)
        {
            // 璇诲彇TIM5璁℃暟鍣ㄥ€�(寰璁℃椂)
            uint32_t now = TIM_GetCounter(TIM5);

            // 闃舵1锛氳繃婊ゅ彂灏勮繎绔洸鍖猴紝婊ら櫎鍙戝皠鑰﹀悎骞叉壈 (鎺㈠ご鍙戝皠鍚庣殑浣欓渿鏈�)
            if(now >= ULTRASONIC_BLANKING_US)
            {
                // 寮曡剼涓洪珮鐢靛钩锛氬垽瀹氫负銆愪笂鍗囨部銆�
                if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) != Bit_RESET)
                {
                    // 鏈褰曡繃涓婂崌娌� + 鍦ㄦ湁鏁堟椂闂寸獥鍙ｅ唴锛岃褰曚笂鍗囨部鏃跺埢
                    if((g_echo_rise_seen == 0U) && (now >= g_echo_accept_min_us))
                    {
                        g_echo_rise_us = now;     // 淇濆瓨涓婂崌娌挎椂闂存埑
                        g_echo_rise_seen = 1U;    // 鏍囪宸叉崟鑾蜂笂鍗囨部
                    }
                }
                // 寮曡剼涓轰綆鐢靛钩 + 宸叉崟鑾蜂笂鍗囨部 + 鏃堕棿鍚堟硶锛氬垽瀹氫负銆愪笅闄嶆部銆�
                else if(g_echo_rise_seen != 0U && now > g_echo_rise_us)
                {
                    g_echo_fall_us = now;  // 淇濆瓨涓嬮檷娌挎椂闂存埑
                    // 璁＄畻鍥炴尝宄板€肩瓑鏁堟椂闂�(涓績鏃跺埢绠楁硶)
                    g_echo_time_us = Ultrasonic_EstimatePeakTime(g_echo_rise_us, g_echo_fall_us);

                    // 澶氶噸鏈夋晥鎬ф牎楠岋細鏃堕棿鑼冨洿銆佽剦鍐插搴︺€佺獥鍙ｈ寖鍥达紝婊ら櫎姣涘埡/骞叉壈
                    if((g_echo_time_us >= ULTRASONIC_MIN_VALID_US) &&
                       ((g_echo_fall_us - g_echo_rise_us) >= ULTRASONIC_MIN_PULSE_WIDTH_US) &&
                       (g_echo_time_us >= g_echo_accept_min_us) &&
                       (g_echo_time_us <= g_echo_accept_max_us))
                    {
                        g_echo_captured = 1U;    // 鏍囪閲囨牱瀹屾垚锛屼富寰幆鍙彇鏁�
                        g_measure_active = 0U;   // 鍏抽棴鏈娴嬮噺绐楀彛锛屽仠姝㈡帴鏀朵腑鏂�
                    }
                    // 鏍￠獙澶辫触锛氭竻绌虹姸鎬侊紝绛夊緟涓嬩竴娆″洖娉�
                    else
                    {
                        g_echo_rise_seen = 0U;
                        g_echo_rise_us = 0U;
                        g_echo_fall_us = 0U;
                    }
                }
            }
        }

        // 蹇呴』娓呴櫎涓柇鎸傝捣鏍囧織锛屽惁鍒欎細閲嶅杩涗腑鏂�
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
