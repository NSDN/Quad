#include "LCD.h"

void main() {
	LCD_RS = 1; LCD_RW = 1; LCD_E = 1;
	LCD_DATA = 0xFF;
	
	lcdInit();
	lcdClear();
	
	lcdPrint(0, 0, "Hello World!");
	lcdPrint(0, 1, "Hello Gensokyo!");
	
	while (1);
}
