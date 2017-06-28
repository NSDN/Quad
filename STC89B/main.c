#include "LCD.h"

void main() {		
	lcdInit();
	lcdClear();
	
	lcdPrint(0, 0, "Hello World!");
	lcdPrint(0, 1, "Hello Gensokyo!");
	
	while (1);
}
