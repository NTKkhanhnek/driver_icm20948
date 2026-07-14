#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include <stdint.h>

#define ICM20948_REG_WHO_AM_I            0x00
#define ICM20948_REG_USER_CTRL           0x03
#define ICM20948_REG_PWR_MGMT_1          0x06
#define ICM20948_REG_PWR_MGMT_2          0x07
#define ICM20948_REG_ACCEL_XOUT_H        0x2D
#define ICM20948_REG_REG_BANK_SEL        0x7F

#define ICM20948_REG_GYRO_SMPLRT_DIV     0x00
#define ICM20948_REG_GYRO_CONFIG_1       0x01
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20948_REG_ACCEL_CONFIG        0x14

#define ICM20948_WHO_AM_I_VALUE          0xEA
#define ICM20948_SPI_READ_BIT            0x80
#define ICM20948_SPI_WRITE_BIT           0x00

typedef enum
{
    ICM20948_OK = 0,
    ICM20948_ERROR,
    ICM20948_BUSY,
    ICM20948_TIMEOUT
} ICM20948_StatusTypeDef;

typedef enum
{
    ICM20948_ACCEL_FS_2G  = 0,
    ICM20948_ACCEL_FS_4G  = 1,
    ICM20948_ACCEL_FS_8G  = 2,
    ICM20948_ACCEL_FS_16G = 3
} ICM20948_AccelFullScale_t;

typedef enum
{
    ICM20948_GYRO_FS_250DPS  = 0,
    ICM20948_GYRO_FS_500DPS  = 1,
    ICM20948_GYRO_FS_1000DPS = 2,
    ICM20948_GYRO_FS_2000DPS = 3
} ICM20948_GyroFullScale_t;

typedef enum
{
    ICM20948_ODR_DIV_1   = 0,
    ICM20948_ODR_DIV_2   = 1,
    ICM20948_ODR_DIV_5   = 4,
    ICM20948_ODR_DIV_10  = 9,
    ICM20948_ODR_DIV_20  = 19,
    ICM20948_ODR_DIV_50  = 49,
    ICM20948_ODR_DIV_100 = 99
} ICM20948_ODR_t;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} ICM20948_RawData_t;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
} ICM20948_Data_t;

ICM20948_StatusTypeDef ICM20948_Init(void);
ICM20948_StatusTypeDef ICM20948_ReadWhoAmI(uint8_t *who_am_i);
ICM20948_StatusTypeDef ICM20948_SetAccelConfig(ICM20948_AccelFullScale_t full_scale, ICM20948_ODR_t odr);
ICM20948_StatusTypeDef ICM20948_SetGyroConfig(ICM20948_GyroFullScale_t full_scale, ICM20948_ODR_t odr);
ICM20948_StatusTypeDef ICM20948_ReadRawData(ICM20948_RawData_t *raw_data);
ICM20948_StatusTypeDef ICM20948_ConvertRawToSensorData(const ICM20948_RawData_t *raw_data, ICM20948_Data_t *sensor_data);
ICM20948_StatusTypeDef ICM20948_ReadSensorData(ICM20948_Data_t *sensor_data);

#endif /* INC_ICM20948_H_ */
