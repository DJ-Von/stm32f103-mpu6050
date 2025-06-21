bool mpu6050_init(void);
bool mpu6050_write_reg(uint8_t reg, uint8_t value);
bool mpu6050_read_regs(uint8_t start_reg, uint8_t* buffer, uint16_t len);
void mpu6050_calibrate(void);
bool mpu6050_read_raw(int16_t* accel, int16_t* gyro);
void mpu6050_get_data(float* accel_g, float* gyro_dps);
void mpu6050_get_angles(float* roll, float* pitch, float dt);
