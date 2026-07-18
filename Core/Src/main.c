#include "delay.h"
#include "icm20948.h"
#include "clock.h"

volatile float accel_x_g = 0.0f;
volatile float accel_y_g = 0.0f;
volatile float accel_z_g = 0.0f;
volatile float gyro_x_dps = 0.0f;
volatile float gyro_y_dps = 0.0f;
volatile float gyro_z_dps = 0.0f;
volatile float temperature_c = 0.0f;

volatile uint8_t icm20948_init_ok = 0;
volatile uint8_t icm20948_who_am_i = 0;
volatile uint8_t gyro_calibration_ok = 0;
volatile uint8_t gyro_calibrated = 0;
volatile float gyro_offset_x_raw = 0.0f;
volatile float gyro_offset_y_raw = 0.0f;
volatile float gyro_offset_z_raw = 0.0f;

extern volatile uint8_t debug_icm20948_gyro_calibrated;

int main(void)
{
    uint8_t who_am_i;
    ICM20948_GyroCalibration_t gyro_calibration;

    clock_init();
    delay_init(RCC_SYS_CLOCK_HZ);

    icm20948_init_ok = (ICM20948_Init() == ICM20948_OK);
    if (ICM20948_ReadWhoAmI(&who_am_i) == ICM20948_OK)
    {
        icm20948_who_am_i = who_am_i;
    }

    if (icm20948_init_ok != 0U)
    {
        gyro_calibration_ok = (ICM20948_CalibrateGyro(500) == ICM20948_OK);
        ICM20948_GetGyroCalibration(&gyro_calibration);
        gyro_offset_x_raw = gyro_calibration.offset_x_raw;
        gyro_offset_y_raw = gyro_calibration.offset_y_raw;
        gyro_offset_z_raw = gyro_calibration.offset_z_raw;
    }

    while (1)
    {
        ICM20948_Data_t sensor_data;

        if (ICM20948_ReadSensorData(&sensor_data) == ICM20948_OK)
        {
            accel_x_g = sensor_data.accel_x_g;
            accel_y_g = sensor_data.accel_y_g;
            accel_z_g = sensor_data.accel_z_g;
            gyro_x_dps = sensor_data.gyro_x_dps;
            gyro_y_dps = sensor_data.gyro_y_dps;
            gyro_z_dps = sensor_data.gyro_z_dps;
            temperature_c = sensor_data.temperature_c;
            gyro_calibrated = debug_icm20948_gyro_calibrated;
        }

        delay_ms(10);
    }
}
