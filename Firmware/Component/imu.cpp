#include "imu.hpp"
#include "bmi088_probe.h"

const float imu_k[3] = {16.0f * 9.8f / 32768.0f, 2000.0f / 32768.0f, 180.0f / 32768.0f};

void IMU::decode(uint8_t raw_data[IMU_RX_DATA_LENGTH])
{
    for (uint8_t j = 0; j < 33; j++)
    {
        if (raw_data[j] != 0x55)
            continue;

        for (uint8_t i = 0; i < 3; i++)
        {
            if (raw_data[j + 0 + i * 11] == 0x55 && raw_data[j + 1 + i * 11] == (0x51 + i))
            {

                if (sumcrc(&(raw_data[j + 0 + i * 11])))
                {
                    data_[0 + i * 4] = (short)(((short)raw_data[j + 3 + i * 11] << 8) | raw_data[j + 2 + i * 11]) * imu_k[i];
                    data_[1 + i * 4] = (short)(((short)raw_data[j + 5 + i * 11] << 8) | raw_data[j + 4 + i * 11]) * imu_k[i];
                    data_[2 + i * 4] = (short)(((short)raw_data[j + 7 + i * 11] << 8) | raw_data[j + 6 + i * 11]) * imu_k[i];
                    // data_[kVoltage] = (short)(((short)raw_data[9] << 8) | raw_data[8]) / 100.0;
                }
            }
        }
    }
}

bool IMU::sumcrc(const uint8_t raw_data[11])
{
    uint16_t sum = 0x0;
    for (size_t i = 0; i < 10; i++)
    {
        sum += raw_data[i];
    }
    uint8_t crc = sum & 0xFF;
    return (crc == raw_data[10]);
}

void IMU::get_data(float out_data[9]) const
{
    out_data[0] = data_[kAccX];
    out_data[1] = data_[kAccY];
    out_data[2] = data_[kAccZ];
    // correct mapping: omega X/Y/Z
    out_data[3] = data_[kOmegaX];
    out_data[4] = data_[kOmegaY];
    out_data[5] = data_[kOmegaZ];
    
    out_data[6] = data_[kAngleX];
    out_data[7] = data_[kAngleY];
    out_data[8] = data_[kAngleZ];
}

// BMI088 conversion factors
// Accel: ±24g range, LSB sensitivity = 1/1365 g/LSB, convert to m/s²
// Gyro: ±2000 DPS range, LSB sensitivity = 1/16.384 DPS/LSB
void IMU::decode_bmi088(const bmi088_raw_data_t *acc, const bmi088_raw_data_t *gyro)
{
    if (acc == nullptr || gyro == nullptr) return;
    
    // BMI088 accel: ±24g, 14-bit data (actually 14-bit but stored in 16-bit)
    // Sensitivity: 1/1365 g/LSB at ±24g range
    // Convert to m/s²: LSB * (24 * 9.8 / 32768)
    data_[kAccX] = acc->x * (24.0f * 9.8f / 32768.0f);
    data_[kAccY] = acc->y * (24.0f * 9.8f / 32768.0f);
    data_[kAccZ] = acc->z * (24.0f * 9.8f / 32768.0f);
    
    // BMI088 gyro: ±2000 DPS
    // Sensitivity: 1/16.384 DPS/LSB at ±2000 DPS range
    // Keep as DPS: LSB * (2000 / 32768)
    data_[kOmegaX] = gyro->x * (2000.0f / 32768.0f);
    data_[kOmegaY] = gyro->y * (2000.0f / 32768.0f);
    data_[kOmegaZ] = gyro->z * (2000.0f / 32768.0f);
    
    // Note: BMI088 does not provide Euler angles directly
    // Angles are typically computed by firmware/external algorithm
    // For now, keep them as zeros or indicate they need external computation
    data_[kAngleX] = 0.0f;
    data_[kAngleY] = 0.0f;
    data_[kAngleZ] = 0.0f;
}