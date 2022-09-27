#ifndef _STM32_ICM20948_H_
#define _STM32_ICM20948_H_

/*************************************************************************
  Defines
*************************************************************************/

#include <stdbool.h>
#include <stdio.h>

#define ICM20948_UART   huart4
#define ICM20948_I2C    hi2c1

#define AK0991x_DEFAULT_I2C_ADDR    0x0C    /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E    /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA           0x68    /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB           0x69    /* I2C slave address for INV device on Rev B board */


typedef struct
{
    int mode;
    bool enable_gyroscope;
    bool enable_accelerometer;
    bool enable_magnetometer;
    bool enable_quaternion;
    int gyroscope_frequency;
    int accelerometer_frequency;
    int magnetometer_frequency;
    int quaternion_frequency;

} STM32ICM20948Settings;

/*************************************************************************
  Class
*************************************************************************/

void ICM20948_init(STM32ICM20948Settings settings);
void ICM20948_task();

bool ICM20948_gyroDataIsReady();
bool ICM20948_accelDataIsReady();
bool ICM20948_magDataIsReady();
bool ICM20948_quatDataIsReady();

void ICM20948_readGyroData(float *x, float *y, float *z);
void ICM20948_readAccelData(float *x, float *y, float *z);
void ICM20948_readMagData(float *x, float *y, float *z);
void ICM20948_readQuatData(float *w, float *x, float *y, float *z);

#endif /* _STM32_ICM20948_H_ */