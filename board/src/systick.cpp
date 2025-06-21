#include "systick.h"

extern "C" {
    void sys_tick_handler(void);
}

static volatile uint32_t _millis = 0;

void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(7999);
	systick_interrupt_enable();
	systick_counter_enable();
}

uint32_t millis() {
    return _millis;
}

void sys_tick_handler(void) {
    _millis++;
}

void delay(uint32_t duration) {
    const uint32_t until = millis() + duration;
    while (millis() < until);
}
