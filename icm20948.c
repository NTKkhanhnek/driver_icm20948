#include "icm20948.h"
#include "delay.h"
#include "spi.h"

#define ICM20948_BANK0                 0x00
#define ICM20948_BANK2                 0x02

#define ICM20948_DEVICE_RESET          0x80
#define ICM20948_CLKSEL_AUTO           0x01
#define ICM20948_I2C_IF_DIS            0x10
#define ICM20948_PWR_ALL_ON            0x00

#define ICM20948_GYRO_DLPFCFG_196HZ    0x00
#define ICM20948_ACCEL_DLPFCFG_246HZ   0x00
#define ICM20948_FCHOICE_DLPF_ON       0x01

static ICM20948_AccelFullScale_t accel_full_scale = ICM20948_ACCEL_FS_16G;
static ICM20948_GyroFullScale_t gyro_full_scale = ICM20948_GYRO_FS_2000DPS;

volatile int16_t debug_icm20948_accel_x;
volatile int16_t debug_icm20948_accel_y;
volatile int16_t debug_icm20948_accel_z;
volatile int16_t debug_icm20948_gyro_x;
volatile int16_t debug_icm20948_gyro_y;
volatile int16_t debug_icm20948_gyro_z;
volatile int16_t debug_icm20948_temp;
volatile uint8_t debug_icm20948_who_am_i;

static ICM20948_StatusTypeDef ICM20948_ConvertSPIStatus(SPI_Status_t status)
{
    if (status == SPI_OK)
    {
        return ICM20948_OK;
    }

    if (status == SPI_TIMEOUT)
    {
        return ICM20948_TIMEOUT;
    }

    return ICM20948_ERROR;
}

static ICM20948_StatusTypeDef ICM20948_WriteReg(uint8_t reg, uint8_t data)
{
    return ICM20948_ConvertSPIStatus(SPI_mem_write((uint8_t)(reg | ICM20948_SPI_WRITE_BIT), &data, 1));
}

static ICM20948_StatusTypeDef ICM20948_ReadReg(uint8_t reg, uint8_t *data)
{
    if (data == 0)
    {
        return ICM20948_ERROR;
    }

    return ICM20948_ConvertSPIStatus(SPI_mem_read((uint8_t)(reg | ICM20948_SPI_READ_BIT), data, 1));
}

static ICM20948_StatusTypeDef ICM20948_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if ((data == 0) || (len == 0))
    {
        return ICM20948_ERROR;
    }

    return ICM20948_ConvertSPIStatus(SPI_mem_read((uint8_t)(reg | ICM20948_SPI_READ_BIT), data, len));
}

static ICM20948_StatusTypeDef ICM20948_SelectBank(uint8_t bank)
{
    return ICM20948_WriteReg(ICM20948_REG_REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
}

static ICM20948_StatusTypeDef ICM20948_CheckID(void)
{
    uint8_t who_am_i;

    if (ICM20948_ReadWhoAmI(&who_am_i) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (who_am_i != ICM20948_WHO_AM_I_VALUE)
    {
        return ICM20948_ERROR;
    }

    return ICM20948_OK;
}

static ICM20948_StatusTypeDef ICM20948_Reset(void)
{
    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_PWR_MGMT_1, ICM20948_DEVICE_RESET) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    delay_ms(100);
    return ICM20948_OK;
}

static float ICM20948_GetAccelSensitivity(void)
{
    switch (accel_full_scale)
    {
        case ICM20948_ACCEL_FS_2G:
            return 16384.0f;
        case ICM20948_ACCEL_FS_4G:
            return 8192.0f;
        case ICM20948_ACCEL_FS_8G:
            return 4096.0f;
        case ICM20948_ACCEL_FS_16G:
        default:
            return 2048.0f;
    }
}

static float ICM20948_GetGyroSensitivity(void)
{
    switch (gyro_full_scale)
    {
        case ICM20948_GYRO_FS_250DPS:
            return 131.0f;
        case ICM20948_GYRO_FS_500DPS:
            return 65.5f;
        case ICM20948_GYRO_FS_1000DPS:
            return 32.8f;
        case ICM20948_GYRO_FS_2000DPS:
        default:
            return 16.4f;
    }
}

ICM20948_StatusTypeDef ICM20948_Init(void)
{
    SPI_init();
    delay_ms(100);

    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_CheckID() != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_Reset() != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_CheckID() != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_USER_CTRL, ICM20948_I2C_IF_DIS) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_PWR_MGMT_1, ICM20948_CLKSEL_AUTO) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_PWR_MGMT_2, ICM20948_PWR_ALL_ON) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SetGyroConfig(ICM20948_GYRO_FS_2000DPS, ICM20948_ODR_DIV_1) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SetAccelConfig(ICM20948_ACCEL_FS_16G, ICM20948_ODR_DIV_1) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    delay_ms(50);
    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_ReadWhoAmI(uint8_t *who_am_i)
{
    if (who_am_i == 0)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_ReadReg(ICM20948_REG_WHO_AM_I, who_am_i) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    debug_icm20948_who_am_i = *who_am_i;
    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_SetAccelConfig(ICM20948_AccelFullScale_t full_scale, ICM20948_ODR_t odr)
{
    uint8_t config;

    if (full_scale > ICM20948_ACCEL_FS_16G)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK2) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t)(((uint16_t)odr >> 8) & 0x0F)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t)((uint16_t)odr & 0xFF)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    config = (uint8_t)((ICM20948_ACCEL_DLPFCFG_246HZ << 3) | ((uint8_t)full_scale << 1) | ICM20948_FCHOICE_DLPF_ON);

    if (ICM20948_WriteReg(ICM20948_REG_ACCEL_CONFIG, config) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    accel_full_scale = full_scale;
    return ICM20948_SelectBank(ICM20948_BANK0);
}

ICM20948_StatusTypeDef ICM20948_SetGyroConfig(ICM20948_GyroFullScale_t full_scale, ICM20948_ODR_t odr)
{
    uint8_t config;

    if (full_scale > ICM20948_GYRO_FS_2000DPS)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK2) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_GYRO_SMPLRT_DIV, (uint8_t)((uint16_t)odr & 0xFF)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    config = (uint8_t)((ICM20948_GYRO_DLPFCFG_196HZ << 3) | ((uint8_t)full_scale << 1) | ICM20948_FCHOICE_DLPF_ON);

    if (ICM20948_WriteReg(ICM20948_REG_GYRO_CONFIG_1, config) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    gyro_full_scale = full_scale;
    return ICM20948_SelectBank(ICM20948_BANK0);
}

ICM20948_StatusTypeDef ICM20948_ReadRawData(ICM20948_RawData_t *raw_data)
{
    uint8_t raw[14];

    if (raw_data == 0)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_ReadRegs(ICM20948_REG_ACCEL_XOUT_H, raw, 14) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    raw_data->accel_x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    raw_data->accel_y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    raw_data->accel_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    raw_data->gyro_x = (int16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    raw_data->gyro_y = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    raw_data->gyro_z = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    raw_data->temp = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

    debug_icm20948_accel_x = raw_data->accel_x;
    debug_icm20948_accel_y = raw_data->accel_y;
    debug_icm20948_accel_z = raw_data->accel_z;
    debug_icm20948_gyro_x = raw_data->gyro_x;
    debug_icm20948_gyro_y = raw_data->gyro_y;
    debug_icm20948_gyro_z = raw_data->gyro_z;
    debug_icm20948_temp = raw_data->temp;

    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_ConvertRawToSensorData(const ICM20948_RawData_t *raw_data, ICM20948_Data_t *sensor_data)
{
    float accel_sensitivity;
    float gyro_sensitivity;

    if ((raw_data == 0) || (sensor_data == 0))
    {
        return ICM20948_ERROR;
    }

    accel_sensitivity = ICM20948_GetAccelSensitivity();
    gyro_sensitivity = ICM20948_GetGyroSensitivity();

    sensor_data->accel_x_g = (float)raw_data->accel_x / accel_sensitivity;
    sensor_data->accel_y_g = (float)raw_data->accel_y / accel_sensitivity;
    sensor_data->accel_z_g = (float)raw_data->accel_z / accel_sensitivity;
    sensor_data->gyro_x_dps = (float)raw_data->gyro_x / gyro_sensitivity;
    sensor_data->gyro_y_dps = (float)raw_data->gyro_y / gyro_sensitivity;
    sensor_data->gyro_z_dps = (float)raw_data->gyro_z / gyro_sensitivity;
    sensor_data->temperature_c = ((float)raw_data->temp / 333.87f) + 21.0f;

    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_ReadSensorData(ICM20948_Data_t *sensor_data)
{
    ICM20948_RawData_t raw_data;

    if (sensor_data == 0)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_ReadRawData(&raw_data) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    return ICM20948_ConvertRawToSensorData(&raw_data, sensor_data);
}
