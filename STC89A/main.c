#include "lcd.h"

void main() {
	LCD_RS = 1; LCD_RW = 1;
	LCD_E = 1; LCD_DATA = 0xFF;
	
	while (1);
}
