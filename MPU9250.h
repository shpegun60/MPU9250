/*
 * MPU925.h
 *
 * Created on: 23 ìàÿ 2018 ã.
 *      Author: Max
 *  Modified on May 9,2022
 *      Modified by : zhyf0610
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "main.h"
#include "MPU9250_Config.h"
#include <stdbool.h>

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS,
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G,
} AccelRange;

typedef enum DlpfBandwidth
{
  DLPF_BANDWIDTH_184HZ,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
} DlpfBandwidth;

typedef enum LpAccelOdr
{
  LP_ACCEL_ODR_0_24HZ = 0,
  LP_ACCEL_ODR_0_49HZ = 1,
  LP_ACCEL_ODR_0_98HZ = 2,
  LP_ACCEL_ODR_1_95HZ = 3,
  LP_ACCEL_ODR_3_91HZ = 4,
  LP_ACCEL_ODR_7_81HZ = 5,
  LP_ACCEL_ODR_15_63HZ = 6,
  LP_ACCEL_ODR_31_25HZ = 7,
  LP_ACCEL_ODR_62_50HZ = 8,
  LP_ACCEL_ODR_125HZ = 9,
  LP_ACCEL_ODR_250HZ = 10,
  LP_ACCEL_ODR_500HZ = 11
} LpAccelOdr;

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init();

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range);

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range);

/* sets the DLPF bandwidth to values other than default */
void MPU9250_setDlpfBandwidth(DlpfBandwidth bandwidth);

/* sets the sample rate divider to values other than default */
void MPU9250_setSrd(uint8_t srd);

/* enables the data ready interrupt */
void MPU9250_enableDataReadyInterrupt();

/* disables the data ready interrupt */
void MPU9250_disableDataReadyInterrupt();

/* configures and enables wake on motion, low power mode */
void MPU9250_enableWakeOnMotion(float womThresh_mg, LpAccelOdr odr);

/* read the data, each argiment should point to a array for x, y, and z , at last read the temperature data */
void MPU9250_GetData(float* AccData, float* MagData, float* GyroData, float *TempData);

/* estimates the gyro biases */
void MPU9250_calibrateGyro();

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
void MPU9250_calibrateAccel();
void MPU9250_getCalibrations(float *_gyroBiases, float *_accBiases, float *_accMultiplications);
void MPU9250_setCalibrations(float *_gyroBiases, float *_accBiases, float *_accMultiplications);




/* reads data from the MPU9250 FIFO and stores in buffer ------------------------------------------------------------------------------------- */

/* configures and enables the FIFO buffer  */
void MPU9250FIFO_enableFifo(bool accel,bool gyro,bool mag,bool temp);

void MPU9250FIFO_readFifo();

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU9250FIFO_getFifoAccelX_mss(size_t *size,float* data);
/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU9250FIFO_getFifoAccelY_mss(size_t *size,float* data);
/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU9250FIFO_getFifoAccelZ_mss(size_t *size,float* data);
/* returns the gyroscope FIFO size and data in the x direction, deg/s */
void MPU9250FIFO_getFifoGyroX_rads(size_t *size,float* data);
/* returns the gyroscope FIFO size and data in the y direction, deg/s */
void MPU9250FIFO_getFifoGyroY_rads(size_t *size,float* data);
/* returns the gyroscope FIFO size and data in the z direction, deg/s */
void MPU9250FIFO_getFifoGyroZ_rads(size_t *size,float* data);
/* returns the magnetometer FIFO size and data in the x direction, uT */
void MPU9250FIFO_getFifoMagX_uT(size_t *size,float* data);
/* returns the magnetometer FIFO size and data in the y direction, uT */
void MPU9250FIFO_getFifoMagY_uT(size_t *size,float* data);
/* returns the magnetometer FIFO size and data in the z direction, uT */
void MPU9250FIFO_getFifoMagZ_uT(size_t *size,float* data);
/* returns the die temperature FIFO size and data, C */
void MPU9250FIFO_getFifoTemperature_C(size_t *size,float* data);

#endif /* MPU9250_H_ */





