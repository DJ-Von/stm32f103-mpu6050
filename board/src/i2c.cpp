#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>

#include "i2c.h"
#include "systick.h"

#define I2C_TIMEOUT 100000

void i2c_setup(void) {
    // Сброс
    rcc_periph_reset_pulse(RST_I2C1);

    // Настройка пинов
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    // Включение внутренних подтягивающих резисторов
    gpio_set(GPIOB, GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    i2c_peripheral_disable(I2C1);

    // Настройка
    i2c_set_clock_frequency(I2C1, 36); // APB1 частота 36 МГц
    i2c_set_standard_mode(I2C1);       // 100 kHz
    i2c_set_ccr(I2C1, 180);            // Для 100kHz при 36MHz
    i2c_set_trise(I2C1, 37);           // Для 100kHz

    i2c_peripheral_enable(I2C1);

    // Задержка для стабилизации
    delay(10);
}

bool i2c_wait_for_flag(uint32_t flag) {
    for (uint32_t i = 0; i < I2C_TIMEOUT; i++) {
        if (I2C_SR1(I2C1) & flag) {
            return true;
        }
    }
    return false;
}
