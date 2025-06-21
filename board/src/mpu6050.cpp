#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>

#include "i2c.h"
#include "mpu6050.h"
#include "systick.h"

#include <stdio.h>
#include <cmath>

// Адреса и регистры MPU6050
#define MPU6050_ADDR          0x68
#define MPU6050_REG_PWR_MGMT1 0x6B
#define MPU6050_REG_WHO_AM_I  0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

// Конфигурационные регистры MPU6050
#define MPU6050_REG_SMPLRT_DIV   0x19   // Sample Rate Divider
#define MPU6050_REG_CONFIG       0x1A   // Configuration
#define MPU6050_REG_GYRO_CONFIG  0x1B   // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG 0x1C   // Accelerometer Configuration
#define MPU6050_REG_INT_PIN_CFG  0x37   // Interrupt Pin Configuration
#define MPU6050_REG_INT_ENABLE   0x38   // Interrupt Enable

// Значения для регистра конфигурации гироскопа (GYRO_CONFIG)
#define MPU6050_GYRO_FS_500  0x08  // ±500 °/s

// Значения для регистра конфигурации акселерометра (ACCEL_CONFIG)
#define MPU6050_ACCEL_FS_4   0x08  // ±4g

// Значения для регистра управления питанием (PWR_MGMT_1)
#define MPU6050_PWR1_DEVICE_RESET 0x80
#define MPU6050_PWR1_CLKSEL_INTER 0x01

// Коэффициенты масштабирования
#define ACCEL_SENSITIVITY 8192.0f    // ±4g: 8192 LSB/g
#define GYRO_SENSITIVITY  65.5f      // ±500°/s: 65.5 LSB/°/s

typedef struct {
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
} mpu6050_calibration_t;

mpu6050_calibration_t calibration = {0};

// Запись регистра с обработкой ошибок
bool mpu6050_write_reg(uint8_t reg, uint8_t value) {
    // Генерация START
    I2C_CR1(I2C1) |= I2C_CR1_START;
    if (!i2c_wait_for_flag(I2C_SR1_SB)) return false;

    // Отправка адреса устройства + запись
    I2C_DR(I2C1) = MPU6050_ADDR << 1;
    if (!i2c_wait_for_flag(I2C_SR1_ADDR)) return false;
    (void)I2C_SR2(I2C1); // Сброс ADDR

    // Отправка номера регистра
    if (!i2c_wait_for_flag(I2C_SR1_TxE)) return false;
    I2C_DR(I2C1) = reg;

    // Отправка значения
    if (!i2c_wait_for_flag(I2C_SR1_TxE)) return false;
    I2C_DR(I2C1) = value;

    // Ожидание завершения передачи
    if (!i2c_wait_for_flag(I2C_SR1_BTF)) return false;

    // Генерация STOP
    I2C_CR1(I2C1) |= I2C_CR1_STOP;

    return true;
}

// Чтение регистров с обработкой ошибок
bool mpu6050_read_regs(uint8_t start_reg, uint8_t* buffer, uint16_t len) {
    // Фаза записи адреса регистра
    I2C_CR1(I2C1) |= I2C_CR1_START;
    if (!i2c_wait_for_flag(I2C_SR1_SB)) return false;

    I2C_DR(I2C1) = MPU6050_ADDR << 1;
    if (!i2c_wait_for_flag(I2C_SR1_ADDR)) return false;
    (void)I2C_SR2(I2C1); // Сброс ADDR

    if (!i2c_wait_for_flag(I2C_SR1_TxE)) return false;
    I2C_DR(I2C1) = start_reg;

    // Повторный START для переключения в режим чтения
    I2C_CR1(I2C1) |= I2C_CR1_START;
    if (!i2c_wait_for_flag(I2C_SR1_SB)) return false;

    // Отправка адреса устройства + чтение
    I2C_DR(I2C1) = (MPU6050_ADDR << 1) | 0x01;
    if (!i2c_wait_for_flag(I2C_SR1_ADDR)) return false;
    (void)I2C_SR2(I2C1); // Сброс ADDR

    // Настройка приема
    if (len > 1) {
        I2C_CR1(I2C1) |= I2C_CR1_ACK;
    } else {
        I2C_CR1(I2C1) &= ~I2C_CR1_ACK;
        I2C_CR1(I2C1) |= I2C_CR1_STOP;
    }

    // Чтение данных
    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            // Последний байт
            I2C_CR1(I2C1) &= ~I2C_CR1_ACK;
            I2C_CR1(I2C1) |= I2C_CR1_STOP;
        }

        if (!i2c_wait_for_flag(I2C_SR1_RxNE)) return false;
        buffer[i] = I2C_DR(I2C1);
    }

    return true;
}

void mpu6050_calibrate(void) {
    printf("Starting calibration... keep sensor still!\r\n");
    delay(2000);

    const int samples = 500;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < samples; i++) {
        int16_t accel_raw[3], gyro_raw[3];
        if (mpu6050_read_raw(accel_raw, gyro_raw)) {
            ax_sum += accel_raw[0];
            ay_sum += accel_raw[1];
            az_sum += accel_raw[2];
            gx_sum += gyro_raw[0];
            gy_sum += gyro_raw[1];
            gz_sum += gyro_raw[2];
        }
        delay(10);
    }

    // Смещения акселерометра
    calibration.ax_offset = ax_sum / samples;
    calibration.ay_offset = ay_sum / samples;
    calibration.az_offset = (az_sum / samples) - ACCEL_SENSITIVITY; // Учет 1g

    // Смещения гироскопа
    calibration.gx_offset = gx_sum / samples;
    calibration.gy_offset = gy_sum / samples;
    calibration.gz_offset = gz_sum / samples;

    printf("Calibration complete!\r\n");
    printf("Accel offsets: X=%.1f Y=%.1f Z=%.1f\r\n",
           calibration.ax_offset, calibration.ay_offset, calibration.az_offset);
    printf("Gyro offsets: X=%.1f Y=%.1f Z=%.1f\r\n",
           calibration.gx_offset, calibration.gy_offset, calibration.gz_offset);
}

bool mpu6050_init(void) {
    // Сброс устройства
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT1, MPU6050_PWR1_DEVICE_RESET)) {
        printf("MPU6050 reset failed!\r\n");
        return false;
    }
    delay(100);

    // Пробуждение устройства и выбор тактового источника
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT1, MPU6050_PWR1_CLKSEL_INTER)) {
        printf("MPU6050 wakeup failed!\r\n");
        return false;
    }

    // Настройка гироскопа: ±500 °/s
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500)) {
        printf("Gyro config failed!\r\n");
        return false;
    }

    // Настройка акселерометра: ±4g
    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_4)) {
        printf("Accel config failed!\r\n");
        return false;
    }

    // Настройка DLPF (цифровой фильтр низких частот)
    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03)) { // ~42Hz bandwidth
        printf("DLPF config failed!\r\n");
        return false;
    }

    // Настройка частоты дискретизации
    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x04)) { // 1kHz/(1+4) = 200Hz
        printf("Sample rate config failed!\r\n");
        return false;
    }

    // Отключение прерываний
    if (!mpu6050_write_reg(MPU6050_REG_INT_ENABLE, 0x00)) {
        printf("Interrupt disable failed!\r\n");
        return false;
    }

    // Проверка идентификатора
    uint8_t who_am_i;
    if (!mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) {
        printf("WHO_AM_I read failed!\r\n");
        return false;
    }
    printf("MPU6050 WHO_AM_I: 0x%02X\r\n", who_am_i);

    return (who_am_i == 0x68);
}

// Чтение сырых данных (акселерометр + гироскоп)
bool mpu6050_read_raw(int16_t* accel, int16_t* gyro) {
    uint8_t data[14];

    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, data, 14)) {
        return false;
    }

    accel[0] = (int16_t)((data[0] << 8) | data[1]);
    accel[1] = (int16_t)((data[2] << 8) | data[3]);
    accel[2] = (int16_t)((data[4] << 8) | data[5]);

    gyro[0] = (int16_t)((data[8] << 8) | data[9]);
    gyro[1] = (int16_t)((data[10] << 8) | data[11]);
    gyro[2] = (int16_t)((data[12] << 8) | data[13]);

    return true;
}

// Преобразование сырых значений в физические величины
void mpu6050_get_data(float* accel_g, float* gyro_dps) {
    int16_t accel_raw[3], gyro_raw[3];

    if (!mpu6050_read_raw(accel_raw, gyro_raw)) {
        // Обработка ошибки
        for (int i = 0; i < 3; i++) {
            accel_g[i] = 0.0f;
            gyro_dps[i] = 0.0f;
        }
        return;
    }

    // Применение калибровки и масштабирование
    accel_g[0] = (accel_raw[0] - calibration.ax_offset) / ACCEL_SENSITIVITY;
    accel_g[1] = (accel_raw[1] - calibration.ay_offset) / ACCEL_SENSITIVITY;
    accel_g[2] = (accel_raw[2] - calibration.az_offset) / ACCEL_SENSITIVITY;

    gyro_dps[0] = (gyro_raw[0] - calibration.gx_offset) / GYRO_SENSITIVITY;
    gyro_dps[1] = (gyro_raw[1] - calibration.gy_offset) / GYRO_SENSITIVITY;
    gyro_dps[2] = (gyro_raw[2] - calibration.gz_offset) / GYRO_SENSITIVITY;
}
