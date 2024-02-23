/*
	Это не пример, тест сваливания в hardfault (в текущей версии при этом должен моргать красный диод)
*/
#include "core.h"

void core_init( void ){
	__ASM volatile("MOVS r0, #1");
	__ASM volatile("MOVS r1, #2");
	__ASM volatile("SVC #0");
}

void core_loop( void ){

}
