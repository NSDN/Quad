#include "STC15.h"

void delay(unsigned short t) {
	//32MHz@1ms
	unsigned char i, j;
	while (--t) {
		i = 32; j = 29;
		do {
			while (--j);
		} while (--i);
	}
}

void main() {
	P0M0 = 0x00; P0M1 = 0x00;
	P3M0 = 0x00; P3M1 = 0x88;
	P5M0 = 0x00; P5M1 = 0x20;
	
	P32 = 1;
	delay(500);
	P32 = 0;
	
	while (1);
}