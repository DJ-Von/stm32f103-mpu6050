#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

void uart_setup() {
    rcc_periph_clock_enable(RCC_USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);
    gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIO_BANK_USART1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART1_RX);
    gpio_set(GPIOA, GPIO_USART1_TX);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    USART_CR1(USART1) |= USART_CR1_RXNEIE;
    usart_enable(USART1);
}

extern "C" {
    ssize_t _write(int file, const char *ptr, ssize_t len);
}

int _write(int file, const char *ptr, ssize_t len) {
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        errno = EIO;
        return -1;
    }
    int i;
    for (i = 0; i < len; i++) {
        usart_send_blocking(USART1, ptr[i]);
    }
    return i;
}
