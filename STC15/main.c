#include "STC15.h"

void main() {
	P0M0 = 0x00; P0M1 = 0x00;
	P3M0 = 0x00; P3M1 = 0x88;
	P5M0 = 0x00; P5M1 = 0x20;
	
	while (1);
}