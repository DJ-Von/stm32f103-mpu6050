#include <stdint.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

void systick_setup(void);
void delay(uint32_t duration);
uint32_t millis(void);
