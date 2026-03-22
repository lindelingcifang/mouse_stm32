#include "bmi088_probe.h"

#include "main.h"
#include "gpio.h"
#include "spi.h"
#include <stdint.h>

#define BMI088_SPI_HANDLE (&hspi2)

#define BMI088_ACC_CS_PORT GPIOB
#define BMI088_ACC_CS_PIN  GPIO_PIN_9
#define BMI088_GYRO_CS_PORT GPIOB
#define BMI088_GYRO_CS_PIN  GPIO_PIN_1

#define BMI088_SPI_READ_MASK 0x80U
#define BMI088_SPI_WRITE_MASK 0x7FU

#define BMI088_ACC_SOFTRESET_REG 0x7EU
#define BMI088_ACC_PWR_CONF_REG  0x7CU
#define BMI088_ACC_PWR_CTRL_REG  0x7DU
#define BMI088_ACC_CONF_REG      0x40U
#define BMI088_ACC_X_LSB_REG     0x12U
#define BMI088_ACC_SENSORTIME0_REG 0x18U

#define BMI088_GYRO_SOFTRESET_REG 0x14U
#define BMI088_GYRO_RANGE_REG     0x0FU
#define BMI088_GYRO_BW_REG        0x10U
#define BMI088_GYRO_LPM1_REG      0x11U
#define BMI088_GYRO_X_LSB_REG     0x02U

#define BMI088_SOFTRESET_CMD 0xB6U

#define BMI088_ACC_BW_NORMAL_BITS    0xA0U
#define BMI088_ACC_ODR_1600_BITS     0x0CU
#define BMI088_GYRO_ODR_2000_BITS    0x01U
#define BMI088_GYRO_RANGE_2000DPS    0x00U

static uint8_t bmi088_xfer_ok = 1U;

static HAL_StatusTypeDef bmi088_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return HAL_SPI_TransmitReceive(BMI088_SPI_HANDLE, (uint8_t *)tx, rx, len, 100);
}

static uint8_t bmi088_spi_rw(uint8_t tx)
{
    uint8_t rx = 0U;
    if (bmi088_spi_transfer(&tx, &rx, 1) != HAL_OK)
    {
        bmi088_xfer_ok = 0U;
    }
    return rx;
}

static void bmi088_select(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void bmi088_deselect(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static uint8_t bmi088_read_regs_acc(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    bmi088_xfer_ok = 1U;

    bmi088_select(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN);
    (void)bmi088_spi_rw((uint8_t)(reg | BMI088_SPI_READ_MASK));
    /* Accel SPI read has one dummy byte before valid payload. */
    (void)bmi088_spi_rw(0x00U);

    for (i = 0; i < len; i++) {
        buf[i] = bmi088_spi_rw(0x00U);
    }

    bmi088_deselect(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN);
    return bmi088_xfer_ok;
}

static uint8_t bmi088_read_regs_gyro(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    bmi088_xfer_ok = 1U;

    bmi088_select(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
    (void)bmi088_spi_rw((uint8_t)(reg | BMI088_SPI_READ_MASK));

    for (i = 0; i < len; i++) {
        buf[i] = bmi088_spi_rw(0x00U);
    }

    bmi088_deselect(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
    return bmi088_xfer_ok;
}

static uint8_t bmi088_write_reg_acc(uint8_t reg, uint8_t value)
{
    bmi088_xfer_ok = 1U;
    bmi088_select(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN);
    (void)bmi088_spi_rw((uint8_t)(reg & BMI088_SPI_WRITE_MASK));
    (void)bmi088_spi_rw(value);
    bmi088_deselect(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN);
    return bmi088_xfer_ok;
}

static uint8_t bmi088_write_reg_gyro(uint8_t reg, uint8_t value)
{
    bmi088_xfer_ok = 1U;
    bmi088_select(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
    (void)bmi088_spi_rw((uint8_t)(reg & BMI088_SPI_WRITE_MASK));
    (void)bmi088_spi_rw(value);
    bmi088_deselect(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN);
    return bmi088_xfer_ok;
}

uint8_t bmi088_read_acc_id(void)
{
    uint8_t id = 0U;
    if (bmi088_read_regs_acc(BMI088_ACC_CHIP_ID_REG, &id, 1) == 0U)
    {
        return 0xFFU;
    }
    return id;
}

uint8_t bmi088_read_gyro_id(void)
{
    uint8_t id = 0U;
    if (bmi088_read_regs_gyro(BMI088_GYRO_CHIP_ID_REG, &id, 1) == 0U)
    {
        return 0xFFU;
    }
    return id;
}

void bmi088_probe_ids(uint8_t *acc_id, uint8_t *gyro_id)
{
    if (acc_id != 0) {
        *acc_id = bmi088_read_acc_id();
    }

    if (gyro_id != 0) {
        *gyro_id = bmi088_read_gyro_id();
    }
}

uint8_t bmi088_init_minimal(void)
{
    uint8_t acc_id;
    uint8_t gyro_id;

    /* Bring both devices to known power states with minimal writes. */
    if (bmi088_write_reg_acc(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_CMD) == 0U)
    {
        return 0U;
    }
    HAL_Delay(2);
    if (bmi088_write_reg_acc(BMI088_ACC_PWR_CONF_REG, 0x00U) == 0U)
    {
        return 0U;
    }
    HAL_Delay(1);
    if (bmi088_write_reg_acc(BMI088_ACC_PWR_CTRL_REG, 0x04U) == 0U)
    {
        return 0U;
    }
    HAL_Delay(5);

    if (bmi088_write_reg_gyro(BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD) == 0U)
    {
        return 0U;
    }
    HAL_Delay(35);
    if (bmi088_write_reg_gyro(BMI088_GYRO_LPM1_REG, 0x00U) == 0U)
    {
        return 0U;
    }
    HAL_Delay(5);

    /* High-rate configuration: Accel 1600Hz, Gyro 2000Hz. */
    if (bmi088_write_reg_acc(BMI088_ACC_CONF_REG, (uint8_t)(BMI088_ACC_BW_NORMAL_BITS | BMI088_ACC_ODR_1600_BITS)) == 0U)
    {
        return 0U;
    }
    HAL_Delay(1);

    if (bmi088_write_reg_gyro(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS) == 0U)
    {
        return 0U;
    }

    if (bmi088_write_reg_gyro(BMI088_GYRO_BW_REG, BMI088_GYRO_ODR_2000_BITS) == 0U)
    {
        return 0U;
    }
    HAL_Delay(1);

    acc_id = bmi088_read_acc_id();
    gyro_id = bmi088_read_gyro_id();

    if ((acc_id == BMI088_ACC_CHIP_ID) && (gyro_id == BMI088_GYRO_CHIP_ID)) {
        return 1U;
    }

    return 0U;
}

uint8_t bmi088_read_raw(bmi088_raw_data_t *acc, bmi088_raw_data_t *gyro)
{
    uint8_t acc_buf[6] = {0};
    uint8_t gyro_buf[6] = {0};

    if ((acc == 0) || (gyro == 0)) {
        return 0U;
    }

    if (bmi088_read_regs_acc(BMI088_ACC_X_LSB_REG, acc_buf, 6) == 0U)
    {
        return 0U;
    }

    if (bmi088_read_regs_gyro(BMI088_GYRO_X_LSB_REG, gyro_buf, 6) == 0U)
    {
        return 0U;
    }

    acc->x = (int16_t)(((uint16_t)acc_buf[1] << 8) | acc_buf[0]);
    acc->y = (int16_t)(((uint16_t)acc_buf[3] << 8) | acc_buf[2]);
    acc->z = (int16_t)(((uint16_t)acc_buf[5] << 8) | acc_buf[4]);

    gyro->x = (int16_t)(((uint16_t)gyro_buf[1] << 8) | gyro_buf[0]);
    gyro->y = (int16_t)(((uint16_t)gyro_buf[3] << 8) | gyro_buf[2]);
    gyro->z = (int16_t)(((uint16_t)gyro_buf[5] << 8) | gyro_buf[4]);

    return 1U;
}

uint8_t bmi088_read_acc_sensor_time(uint32_t *sensor_time)
{
    uint8_t buf[3] = {0};

    if (sensor_time == 0)
    {
        return 0U;
    }

    if (bmi088_read_regs_acc(BMI088_ACC_SENSORTIME0_REG, buf, 3) == 0U)
    {
        return 0U;
    }

    *sensor_time = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
    return 1U;
}
