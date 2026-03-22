#ifndef BMI088_PROBE_H
#define BMI088_PROBE_H

#include <stdint.h>

#define BMI088_ACC_CHIP_ID_REG  0x00U
#define BMI088_GYRO_CHIP_ID_REG 0x00U
#define BMI088_ACC_CHIP_ID      0x1EU
#define BMI088_GYRO_CHIP_ID     0x0FU

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} bmi088_raw_data_t;

uint8_t bmi088_read_acc_id(void);
uint8_t bmi088_read_gyro_id(void);
void bmi088_probe_ids(uint8_t *acc_id, uint8_t *gyro_id);
uint8_t bmi088_init_minimal(void);
uint8_t bmi088_read_raw(bmi088_raw_data_t *acc, bmi088_raw_data_t *gyro);
uint8_t bmi088_read_acc_sensor_time(uint32_t *sensor_time);

#endif
