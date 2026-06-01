#ifndef USER_H
#define USER_H

#include "User_header.h"

// 屏幕显示汉字宏定义（已替换为标准简体中文）
#define TITLE_STR        "超声波测距系统"
#define MODEL_VER_STR    "STM32F407"
#define USER_VER_STR     "V1.0版本"

#define MENU_CHOICE_NUM  3
#define MENU1_CHOICE1    "1 实时测量"
#define MENU1_CHOICE2    "2 参数校准"
#define MENU1_CHOICE3    "3 系统状态"

void User_main(void);

#endif