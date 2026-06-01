#include "Character.h"

static void Draw_Ascii_String(uint16_t y, uint16_t x, uint8_t *str, uint8_t size)
{
    uint16_t cursor = x;

    while(*str != 0U)
    {
        if(*str < 0x80U)
        {
            LCD_ShowChar(cursor, y, *str);
            cursor = (uint16_t)(cursor + size / 2U);
            str++;
        }
        else
        {
            LCD_ShowChar(cursor, y, '?');
            cursor = (uint16_t)(cursor + size / 2U);
            str += 2;
        }
    }
}

void LCD_GB4848(uint16_t y, uint16_t x, uint8_t font[2], uint8_t size)
{
    (void)font;
    LCD_DrawRect(x, y, size, size);
}

void Show_Str48(uint16_t y, uint16_t x, uint8_t *str, uint8_t size)
{
    Draw_Ascii_String(y, x, str, size);
}

void LCD_GB3232(uint16_t y, uint16_t x, uint8_t font[2], uint8_t size)
{
    (void)font;
    LCD_DrawRect(x, y, size, size);
}

void Show_Str32(uint16_t y, uint16_t x, uint8_t *str, uint8_t size)
{
    Draw_Ascii_String(y, x, str, size);
}

void LCD_GB1616(uint16_t y, uint16_t x, uint8_t font[2], uint8_t size)
{
    (void)font;
    LCD_DrawRect(x, y, size, size);
}

void Show_Str(uint16_t y, uint16_t x, uint8_t *str, uint8_t size)
{
    Draw_Ascii_String(y, x, str, size);
}
