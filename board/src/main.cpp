#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include "systick.h"
#include "uart.h"
#include "i2c.h"
#include "mpu6050.h"

void common_setup(void) {
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
    cm_enable_interrupts();
    systick_setup();

    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_AFIO);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int main(void) {
    common_setup();
    i2c_setup();
    uart_setup();

    printf("System started\r\n");

    if (!mpu6050_init()) {
        printf("MPU6050 initialization failed!\r\n");
        while(1);
    }

    mpu6050_calibrate();

    float accel_g[3];
    float gyro_dps[3];

    while (1) {
        mpu6050_get_data(accel_g, gyro_dps);

        gpio_toggle(GPIOC, GPIO13);

        printf("\033[2J\033[H");  // Очистка экрана (если поддерживается терминалом)
        printf("+--------+-----------+-----------+-----------+\r\n");
        printf("| Sensor |    X      |    Y      |    Z      |\r\n");
        printf("+--------+-----------+-----------+-----------+\r\n");
        printf("| Accel  | %8.4f  | %8.4f  | %8.4f  |\r\n", accel_g[0], accel_g[1], accel_g[2]);
        printf("| Gyro   | %8.4f  | %8.4f  | %8.4f  |\r\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("+--------+-----------+-----------+-----------+\r\n");

        delay(100);
    }

    return 0;
}
