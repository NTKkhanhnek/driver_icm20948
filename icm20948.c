#include "icm20948.h"
#include "delay.h"
#include "spi.h"

#define ICM20948_BANK0                 0x00
#define ICM20948_BANK2                 0x02
#define ICM20948_BANK3                 0x03

#define ICM20948_DEVICE_RESET          0x80
#define ICM20948_CLKSEL_AUTO           0x01
#define ICM20948_I2C_IF_DIS            0x10
#define ICM20948_I2C_MST_EN            0x20
#define ICM20948_I2C_MST_RST           0x02
#define ICM20948_PWR_ALL_ON            0x00

#define ICM20948_GYRO_DLPFCFG_196HZ    0x00
#define ICM20948_ACCEL_DLPFCFG_246HZ   0x00
#define ICM20948_FCHOICE_DLPF_ON       0x01

#define ICM20948_REG_EXT_SLV_SENS_DATA_00  0x3B
#define ICM20948_REG_I2C_MST_STATUS        0x17
#define ICM20948_REG_I2C_MST_CTRL          0x01
#define ICM20948_REG_I2C_SLV0_ADDR         0x03
#define ICM20948_REG_I2C_SLV0_REG          0x04
#define ICM20948_REG_I2C_SLV0_CTRL         0x05
#define ICM20948_REG_I2C_SLV0_DO           0x06

#define ICM20948_I2C_SLV_READ          0x80
#define ICM20948_I2C_SLV_EN            0x80
#define ICM20948_I2C_MST_CLK_345KHZ    0x07
#define ICM20948_I2C_MST_LOST_ARB      0x20
#define ICM20948_I2C_MST_SLV0_NACK     0x01

#define AK09916_I2C_ADDR               0x0C
#define AK09916_REG_WIA1               0x00
#define AK09916_REG_WIA2               0x01
#define AK09916_REG_ST1                0x10
#define AK09916_REG_HXL                0x11
#define AK09916_REG_ST2                0x18
#define AK09916_REG_CNTL2              0x31
#define AK09916_REG_CNTL3              0x32
#define AK09916_WIA1_VALUE             0x48
#define AK09916_WIA2_VALUE             0x09
#define AK09916_MODE_POWER_DOWN        0x00
#define AK09916_MODE_CONT_100HZ        0x08
#define AK09916_RESET                  0x01
#define AK09916_ST1_DRDY               0x01
#define AK09916_ST2_HOFL               0x08
#define AK09916_SENSITIVITY_UT_PER_LSB 0.15f
#define AK09916_DATA_READ_LEN          8
#define AK09916_DRDY_RETRY_COUNT       5U
#define AK09916_DRDY_RETRY_DELAY_MS    3U

#define ICM20948_MAG_LPF_ALPHA         0.25f
#define ICM20948_MAG_CAL_MIN_SAMPLES   200U
#define ICM20948_MAG_CAL_MIN_SPAN_UT   20.0f

static ICM20948_AccelFullScale_t accel_full_scale = ICM20948_ACCEL_FS_16G;
static ICM20948_GyroFullScale_t gyro_full_scale = ICM20948_GYRO_FS_2000DPS;
static ICM20948_GyroCalibration_t gyro_calibration = {0.0f, 0.0f, 0.0f, 0U};
static ICM20948_AccelCalibration_t accel_calibration = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0U};
static ICM20948_MagCalibration_t mag_calibration = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0U};
static uint8_t mag_filter_initialized = 0U;
static float mag_filtered_x_ut = 0.0f;
static float mag_filtered_y_ut = 0.0f;
static float mag_filtered_z_ut = 0.0f;
static uint8_t mag_calibrating = 0U;
static uint32_t mag_cal_samples = 0U;
static float mag_cal_min_x_ut = 0.0f;
static float mag_cal_min_y_ut = 0.0f;
static float mag_cal_min_z_ut = 0.0f;
static float mag_cal_max_x_ut = 0.0f;
static float mag_cal_max_y_ut = 0.0f;
static float mag_cal_max_z_ut = 0.0f;

volatile int16_t debug_icm20948_accel_x;
volatile int16_t debug_icm20948_accel_y;
volatile int16_t debug_icm20948_accel_z;
volatile int16_t debug_icm20948_gyro_x;
volatile int16_t debug_icm20948_gyro_y;
volatile int16_t debug_icm20948_gyro_z;
volatile int16_t debug_icm20948_mag_x;
volatile int16_t debug_icm20948_mag_y;
volatile int16_t debug_icm20948_mag_z;
volatile int16_t debug_icm20948_temp;
volatile uint8_t debug_icm20948_who_am_i;
volatile uint8_t debug_ak09916_wia1;
volatile uint8_t debug_ak09916_wia2;
volatile uint8_t debug_icm20948_mag_valid;
volatile uint8_t debug_icm20948_mag_init_ok;
volatile uint8_t debug_icm20948_mag_init_status;
volatile uint8_t debug_icm20948_last_mag_status;
volatile uint8_t debug_icm20948_mag_i2c_status;
volatile uint8_t debug_ak09916_st1;
volatile uint8_t debug_ak09916_st2;
volatile uint8_t debug_icm20948_gyro_calibrated;
volatile uint8_t debug_icm20948_accel_calibrated;

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

static ICM20948_StatusTypeDef ICM20948_ReadAccelGyroTempRaw(ICM20948_RawData_t *raw_data)
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
    raw_data->mag_x = 0;
    raw_data->mag_y = 0;
    raw_data->mag_z = 0;
    raw_data->mag_valid = 0U;

    return ICM20948_OK;
}

static ICM20948_StatusTypeDef ICM20948_InternalI2CWrite(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t i2c_status = 0U;

    if (ICM20948_SelectBank(ICM20948_BANK3) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_ADDR, dev_addr) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_REG, reg) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_DO, data) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_CTRL, (uint8_t)(ICM20948_I2C_SLV_EN | 1U)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    delay_ms(10);

    if (ICM20948_SelectBank(ICM20948_BANK0) == ICM20948_OK)
    {
        if (ICM20948_ReadReg(ICM20948_REG_I2C_MST_STATUS, &i2c_status) == ICM20948_OK)
        {
            debug_icm20948_mag_i2c_status = i2c_status;
        }
    }

    if ((i2c_status & (ICM20948_I2C_MST_LOST_ARB | ICM20948_I2C_MST_SLV0_NACK)) != 0U)
    {
        (void)ICM20948_SelectBank(ICM20948_BANK3);
        (void)ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_CTRL, 0x00);
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK3) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    return ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_CTRL, 0x00);
}

static ICM20948_StatusTypeDef ICM20948_InternalI2CRead(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i2c_status = 0U;

    if ((data == 0) || (len == 0) || (len > 24U))
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK3) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_ADDR, (uint8_t)(dev_addr | ICM20948_I2C_SLV_READ)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_REG, reg) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_CTRL, (uint8_t)(ICM20948_I2C_SLV_EN | len)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    delay_ms(10);

    if (ICM20948_SelectBank(ICM20948_BANK0) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_ReadReg(ICM20948_REG_I2C_MST_STATUS, &i2c_status) == ICM20948_OK)
    {
        debug_icm20948_mag_i2c_status = i2c_status;
    }

    if ((i2c_status & (ICM20948_I2C_MST_LOST_ARB | ICM20948_I2C_MST_SLV0_NACK)) != 0U)
    {
        (void)ICM20948_SelectBank(ICM20948_BANK3);
        (void)ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_CTRL, 0x00);
        (void)ICM20948_SelectBank(ICM20948_BANK0);
        return ICM20948_ERROR;
    }

    if (ICM20948_ReadRegs(ICM20948_REG_EXT_SLV_SENS_DATA_00, data, len) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_SelectBank(ICM20948_BANK3) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_SLV0_CTRL, 0x00) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    return ICM20948_SelectBank(ICM20948_BANK0);
}

static void ICM20948_UpdateMagCalibration(float mag_x_ut, float mag_y_ut, float mag_z_ut)
{
    if (mag_calibrating == 0U)
    {
        return;
    }

    if (mag_cal_samples == 0U)
    {
        mag_cal_min_x_ut = mag_x_ut;
        mag_cal_min_y_ut = mag_y_ut;
        mag_cal_min_z_ut = mag_z_ut;
        mag_cal_max_x_ut = mag_x_ut;
        mag_cal_max_y_ut = mag_y_ut;
        mag_cal_max_z_ut = mag_z_ut;
    }

    if (mag_x_ut < mag_cal_min_x_ut) { mag_cal_min_x_ut = mag_x_ut; }
    if (mag_y_ut < mag_cal_min_y_ut) { mag_cal_min_y_ut = mag_y_ut; }
    if (mag_z_ut < mag_cal_min_z_ut) { mag_cal_min_z_ut = mag_z_ut; }
    if (mag_x_ut > mag_cal_max_x_ut) { mag_cal_max_x_ut = mag_x_ut; }
    if (mag_y_ut > mag_cal_max_y_ut) { mag_cal_max_y_ut = mag_y_ut; }
    if (mag_z_ut > mag_cal_max_z_ut) { mag_cal_max_z_ut = mag_z_ut; }

    mag_cal_samples++;
}

static void ICM20948_ApplyMagCalibration(float *mag_x_ut, float *mag_y_ut, float *mag_z_ut)
{
    if ((mag_x_ut == 0) || (mag_y_ut == 0) || (mag_z_ut == 0) || (mag_calibration.valid == 0U))
    {
        return;
    }

    *mag_x_ut = (*mag_x_ut - mag_calibration.offset_x_ut) * mag_calibration.scale_x;
    *mag_y_ut = (*mag_y_ut - mag_calibration.offset_y_ut) * mag_calibration.scale_y;
    *mag_z_ut = (*mag_z_ut - mag_calibration.offset_z_ut) * mag_calibration.scale_z;
}

static void ICM20948_FilterMag(float *mag_x_ut, float *mag_y_ut, float *mag_z_ut)
{
    if ((mag_x_ut == 0) || (mag_y_ut == 0) || (mag_z_ut == 0))
    {
        return;
    }

    if (mag_filter_initialized == 0U)
    {
        mag_filtered_x_ut = *mag_x_ut;
        mag_filtered_y_ut = *mag_y_ut;
        mag_filtered_z_ut = *mag_z_ut;
        mag_filter_initialized = 1U;
    }
    else
    {
        mag_filtered_x_ut += ICM20948_MAG_LPF_ALPHA * (*mag_x_ut - mag_filtered_x_ut);
        mag_filtered_y_ut += ICM20948_MAG_LPF_ALPHA * (*mag_y_ut - mag_filtered_y_ut);
        mag_filtered_z_ut += ICM20948_MAG_LPF_ALPHA * (*mag_z_ut - mag_filtered_z_ut);
    }

    *mag_x_ut = mag_filtered_x_ut;
    *mag_y_ut = mag_filtered_y_ut;
    *mag_z_ut = mag_filtered_z_ut;
}

static ICM20948_StatusTypeDef ICM20948_InitMagnetometer(void)
{
    uint8_t wia1;
    uint8_t wia2;

    if (ICM20948_SelectBank(ICM20948_BANK3) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_WriteReg(ICM20948_REG_I2C_MST_CTRL, ICM20948_I2C_MST_CLK_345KHZ) != ICM20948_OK)
    {
        debug_icm20948_mag_init_status = 1U;
        return ICM20948_ERROR;
    }

    if (ICM20948_InternalI2CWrite(AK09916_I2C_ADDR, AK09916_REG_CNTL3, AK09916_RESET) != ICM20948_OK)
    {
        debug_icm20948_mag_init_status = 2U;
        return ICM20948_ERROR;
    }
    delay_ms(100);

    if (ICM20948_ReadMagWhoAmI(&wia1, &wia2) != ICM20948_OK)
    {
        debug_icm20948_mag_init_status = 3U;
        return ICM20948_ERROR;
    }

    if ((wia1 != AK09916_WIA1_VALUE) || (wia2 != AK09916_WIA2_VALUE))
    {
        debug_icm20948_mag_init_status = 4U;
        return ICM20948_ERROR;
    }

    if (ICM20948_InternalI2CWrite(AK09916_I2C_ADDR, AK09916_REG_CNTL2, AK09916_MODE_POWER_DOWN) != ICM20948_OK)
    {
        debug_icm20948_mag_init_status = 5U;
        return ICM20948_ERROR;
    }
    delay_ms(10);

    if (ICM20948_InternalI2CWrite(AK09916_I2C_ADDR, AK09916_REG_CNTL2, AK09916_MODE_CONT_100HZ) != ICM20948_OK)
    {
        debug_icm20948_mag_init_status = 6U;
        return ICM20948_ERROR;
    }

    delay_ms(10);
    debug_icm20948_mag_init_ok = 1U;
    debug_icm20948_mag_init_status = 0U;
    return ICM20948_SelectBank(ICM20948_BANK0);
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
    ICM20948_StatusTypeDef mag_status;

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

    if (ICM20948_WriteReg(ICM20948_REG_USER_CTRL, (uint8_t)(ICM20948_I2C_IF_DIS | ICM20948_I2C_MST_RST)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }
    delay_ms(10);

    if (ICM20948_WriteReg(ICM20948_REG_USER_CTRL, (uint8_t)(ICM20948_I2C_IF_DIS | ICM20948_I2C_MST_EN)) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }
    delay_ms(10);

    debug_icm20948_mag_init_ok = 0U;
    debug_icm20948_mag_init_status = 0U;
    mag_status = ICM20948_InitMagnetometer();
    if (mag_status != ICM20948_OK)
    {
        debug_icm20948_last_mag_status = (uint8_t)mag_status;
        return mag_status;
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

ICM20948_StatusTypeDef ICM20948_ReadMagWhoAmI(uint8_t *wia1, uint8_t *wia2)
{
    uint8_t who_am_i[2];

    if ((wia1 == 0) || (wia2 == 0))
    {
        return ICM20948_ERROR;
    }

    if (ICM20948_InternalI2CRead(AK09916_I2C_ADDR, AK09916_REG_WIA1, who_am_i, 2) != ICM20948_OK)
    {
        return ICM20948_ERROR;
    }

    *wia1 = who_am_i[0];
    *wia2 = who_am_i[1];
    debug_ak09916_wia1 = *wia1;
    debug_ak09916_wia2 = *wia2;

    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_ReadMagRaw(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z)
{
    uint8_t raw[AK09916_DATA_READ_LEN];
    uint8_t st1;
    uint8_t st2;
    uint8_t retry;

    if ((mag_x == 0) || (mag_y == 0) || (mag_z == 0))
    {
        return ICM20948_ERROR;
    }

    st1 = 0U;
    for (retry = 0U; retry < AK09916_DRDY_RETRY_COUNT; retry++)
    {
        if (ICM20948_InternalI2CRead(AK09916_I2C_ADDR, AK09916_REG_ST1, &st1, 1) != ICM20948_OK)
        {
            debug_icm20948_last_mag_status = (uint8_t)ICM20948_ERROR;
            return ICM20948_ERROR;
        }

        debug_ak09916_st1 = st1;
        if ((st1 & AK09916_ST1_DRDY) != 0U)
        {
            break;
        }

        delay_ms(AK09916_DRDY_RETRY_DELAY_MS);
    }

    if ((st1 & AK09916_ST1_DRDY) == 0U)
    {
        debug_icm20948_last_mag_status = (uint8_t)ICM20948_BUSY;
        return ICM20948_BUSY;
    }

    if (ICM20948_InternalI2CRead(AK09916_I2C_ADDR, AK09916_REG_HXL, raw, AK09916_DATA_READ_LEN) != ICM20948_OK)
    {
        debug_icm20948_last_mag_status = (uint8_t)ICM20948_ERROR;
        return ICM20948_ERROR;
    }

    st2 = raw[7];
    debug_ak09916_st2 = st2;

    if ((st2 & AK09916_ST2_HOFL) != 0U)
    {
        debug_icm20948_last_mag_status = (uint8_t)ICM20948_BUSY;
        return ICM20948_BUSY;
    }

    *mag_x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    *mag_y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    *mag_z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    debug_icm20948_mag_x = *mag_x;
    debug_icm20948_mag_y = *mag_y;
    debug_icm20948_mag_z = *mag_z;
    debug_icm20948_mag_valid = 1U;
    debug_icm20948_last_mag_status = (uint8_t)ICM20948_OK;

    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_CalibrateGyro(uint16_t sample_count)
{
    ICM20948_RawData_t raw_data;
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    if (sample_count == 0U)
    {
        return ICM20948_ERROR;
    }

    delay_ms(1000);

    for (uint16_t i = 0; i < sample_count; i++)
    {
        if (ICM20948_ReadAccelGyroTempRaw(&raw_data) != ICM20948_OK)
        {
            gyro_calibration.valid = 0U;
            debug_icm20948_gyro_calibrated = 0U;
            return ICM20948_ERROR;
        }

        sum_x += raw_data.gyro_x;
        sum_y += raw_data.gyro_y;
        sum_z += raw_data.gyro_z;
        delay_ms(5);
    }

    gyro_calibration.offset_x_raw = (float)sum_x / (float)sample_count;
    gyro_calibration.offset_y_raw = (float)sum_y / (float)sample_count;
    gyro_calibration.offset_z_raw = (float)sum_z / (float)sample_count;
    gyro_calibration.valid = 1U;
    debug_icm20948_gyro_calibrated = 1U;

    return ICM20948_OK;
}

void ICM20948_SetGyroCalibration(const ICM20948_GyroCalibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    gyro_calibration = *calibration;
    debug_icm20948_gyro_calibrated = gyro_calibration.valid;
}

void ICM20948_GetGyroCalibration(ICM20948_GyroCalibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    *calibration = gyro_calibration;
}

void ICM20948_SetAccelCalibration(const ICM20948_AccelCalibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    accel_calibration = *calibration;
    if ((accel_calibration.scale_x == 0.0f) || (accel_calibration.scale_y == 0.0f) || (accel_calibration.scale_z == 0.0f))
    {
        accel_calibration.scale_x = 1.0f;
        accel_calibration.scale_y = 1.0f;
        accel_calibration.scale_z = 1.0f;
        accel_calibration.valid = 0U;
    }
    debug_icm20948_accel_calibrated = accel_calibration.valid;
}

void ICM20948_GetAccelCalibration(ICM20948_AccelCalibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    *calibration = accel_calibration;
}

void ICM20948_SetMagCalibration(const ICM20948_MagCalibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    mag_calibration = *calibration;
}

void ICM20948_GetMagCalibration(ICM20948_MagCalibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    *calibration = mag_calibration;
}

void ICM20948_StartMagCalibration(void)
{
    mag_calibrating = 1U;
    mag_cal_samples = 0U;
    mag_calibration.valid = 0U;
    mag_calibration.offset_x_ut = 0.0f;
    mag_calibration.offset_y_ut = 0.0f;
    mag_calibration.offset_z_ut = 0.0f;
    mag_calibration.scale_x = 1.0f;
    mag_calibration.scale_y = 1.0f;
    mag_calibration.scale_z = 1.0f;
    mag_filter_initialized = 0U;
    mag_filtered_x_ut = 0.0f;
    mag_filtered_y_ut = 0.0f;
    mag_filtered_z_ut = 0.0f;
}

void ICM20948_StopMagCalibration(void)
{
    float span_x;
    float span_y;
    float span_z;
    float avg_span;

    mag_calibrating = 0U;
    mag_filter_initialized = 0U;

    span_x = mag_cal_max_x_ut - mag_cal_min_x_ut;
    span_y = mag_cal_max_y_ut - mag_cal_min_y_ut;
    span_z = mag_cal_max_z_ut - mag_cal_min_z_ut;

    if ((mag_cal_samples >= ICM20948_MAG_CAL_MIN_SAMPLES) &&
        (span_x >= ICM20948_MAG_CAL_MIN_SPAN_UT) &&
        (span_y >= ICM20948_MAG_CAL_MIN_SPAN_UT) &&
        (span_z >= ICM20948_MAG_CAL_MIN_SPAN_UT))
    {
        avg_span = (span_x + span_y + span_z) / 3.0f;
        mag_calibration.offset_x_ut = (mag_cal_max_x_ut + mag_cal_min_x_ut) * 0.5f;
        mag_calibration.offset_y_ut = (mag_cal_max_y_ut + mag_cal_min_y_ut) * 0.5f;
        mag_calibration.offset_z_ut = (mag_cal_max_z_ut + mag_cal_min_z_ut) * 0.5f;
        mag_calibration.scale_x = avg_span / span_x;
        mag_calibration.scale_y = avg_span / span_y;
        mag_calibration.scale_z = avg_span / span_z;
        mag_calibration.valid = 1U;
    }
    else
    {
        mag_calibration.valid = 0U;
        mag_calibration.offset_x_ut = 0.0f;
        mag_calibration.offset_y_ut = 0.0f;
        mag_calibration.offset_z_ut = 0.0f;
        mag_calibration.scale_x = 1.0f;
        mag_calibration.scale_y = 1.0f;
        mag_calibration.scale_z = 1.0f;
    }
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
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

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
    raw_data->mag_x = 0;
    raw_data->mag_y = 0;
    raw_data->mag_z = 0;
    raw_data->mag_valid = 0U;

    if (ICM20948_ReadMagRaw(&mag_x, &mag_y, &mag_z) == ICM20948_OK)
    {
        raw_data->mag_x = mag_x;
        raw_data->mag_y = mag_y;
        raw_data->mag_z = mag_z;
        raw_data->mag_valid = 1U;
    }
    else
    {
        debug_icm20948_mag_valid = 0U;
    }

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
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;

    if ((raw_data == 0) || (sensor_data == 0))
    {
        return ICM20948_ERROR;
    }

    accel_sensitivity = ICM20948_GetAccelSensitivity();
    gyro_sensitivity = ICM20948_GetGyroSensitivity();

    sensor_data->accel_x_g = (((float)raw_data->accel_x - accel_calibration.offset_x_raw) / accel_sensitivity) * accel_calibration.scale_x;
    sensor_data->accel_y_g = (((float)raw_data->accel_y - accel_calibration.offset_y_raw) / accel_sensitivity) * accel_calibration.scale_y;
    sensor_data->accel_z_g = (((float)raw_data->accel_z - accel_calibration.offset_z_raw) / accel_sensitivity) * accel_calibration.scale_z;
    sensor_data->gyro_x_dps = ((float)raw_data->gyro_x - gyro_calibration.offset_x_raw) / gyro_sensitivity;
    sensor_data->gyro_y_dps = ((float)raw_data->gyro_y - gyro_calibration.offset_y_raw) / gyro_sensitivity;
    sensor_data->gyro_z_dps = ((float)raw_data->gyro_z - gyro_calibration.offset_z_raw) / gyro_sensitivity;
    sensor_data->temperature_c = ((float)raw_data->temp / 333.87f) + 21.0f;
    sensor_data->mag_x_ut = 0.0f;
    sensor_data->mag_y_ut = 0.0f;
    sensor_data->mag_z_ut = 0.0f;
    sensor_data->mag_valid = raw_data->mag_valid;

    if (raw_data->mag_valid != 0U)
    {
        mag_x_ut = (float)raw_data->mag_x * AK09916_SENSITIVITY_UT_PER_LSB;
        mag_y_ut = (float)raw_data->mag_y * AK09916_SENSITIVITY_UT_PER_LSB;
        mag_z_ut = (float)raw_data->mag_z * AK09916_SENSITIVITY_UT_PER_LSB;

        ICM20948_UpdateMagCalibration(mag_x_ut, mag_y_ut, mag_z_ut);
        ICM20948_ApplyMagCalibration(&mag_x_ut, &mag_y_ut, &mag_z_ut);
        ICM20948_FilterMag(&mag_x_ut, &mag_y_ut, &mag_z_ut);

        sensor_data->mag_x_ut = mag_x_ut;
        sensor_data->mag_y_ut = mag_y_ut;
        sensor_data->mag_z_ut = mag_z_ut;
    }

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
