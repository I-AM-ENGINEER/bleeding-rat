#include "core.h"

void core_init( void ){
	__ASM volatile("MOVS r0, #1");
	__ASM volatile("MOVS r1, #2");
	__ASM volatile("SVC #0");
	
	// Ничего инициализировать не надо
}

void core_loop( void ){	
	// RGB, 0...255, загорится красный (канал R = 255, остальные 0)
	ws2812b_write(255,0,0);
	delay(500);
	// RGB, 0...255, загорится зеленый (канал G = 255, остальные 0)
	ws2812b_write(0,255,0);
	delay(500);
	// RGB, 0...255, загорится синий (канал B = 255, остальные 0)
	ws2812b_write(0,0,255);
	delay(500);
	// RGB, 0...255, загорится белый (все каналы 255)
	ws2812b_write(255,255,255);
	delay(500);
	// RGB, 0...255, загорится черный (все каналы 0)
	ws2812b_write(0,0,0);
	delay(3000);
}


