#include "util.h"

#ifndef __LCD_H_
#define __LCD_H_

#define LCD_RS			P32
#define LCD_RW			P33
#define LCD_E			P34
#define LCD_DATA		P1

void lcdInit();
void lcdSetBright(uint8_t value);
void lcdSetContrast(uint8_t value);
void lcdDraw(uint8_t x, uint8_t y, char c);
void lcdPrint(uint8_t x, uint8_t y, char* str);
void lcdClear();

#endif