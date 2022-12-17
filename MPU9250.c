/*
 * MPU9250.c
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 *  Modified on May 9,2022
 *      Modified by : zhyf0610
 */

#include "MPU9250.h"
#include "string.h"
#include "math.h"

#define CALIBRATE_CNT 100


const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint16_t _dev_add = 208;
// 400 kHz
const uint32_t _i2cRate = 400000;

// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t DLPF_184 = 0x01;
const uint8_t DLPF_92 = 0x02;
const uint8_t DLPF_41 = 0x03;
const uint8_t DLPF_20 = 0x04;
const uint8_t DLPF_10 = 0x05;
const uint8_t DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

// MPU9250 in-chip temperature parameters
const uint16_t RoomTemp_Offset = 21 ; 
const float Temp_Sensitivity = 333.87;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;

static uint8_t _buffer[21];

// accel bias and scale factor estimation
float _abias[3]  	= {0.0f, 0.0f, 0.0f};
float _amultiply[3] = {1.0f, 1.0f, 1.0f};
float _axmax, _aymax, _azmax;
float _axmin, _aymin, _azmin;

// gyro bias estimation
float _gbias[3]  = {0.0f, 0.0f, 0.0f};
// scale factors
static float _accelScale;
static float _gyroScale;
static float _mag_adjust[3];
#define     Magnetometer_Sensitivity_Scale_Factor 0.15f


// constants
const float G = 9.807f;

// fifo
bool _enFifoAccel,_enFifoGyro,_enFifoMag,_enFifoTemp;
size_t _fifoSize,_fifoFrameSize;
float _axFifo[85], _ayFifo[85], _azFifo[85];
size_t _aSize;
float _gxFifo[85], _gyFifo[85], _gzFifo[85];
size_t _gSize;
float _hxFifo[73], _hyFifo[73], _hzFifo[73];
size_t _hSize;
float _tFifo[256];
size_t _tSize;

// configuration
AccelRange _accelRange;
GyroRange _gyroRange;
DlpfBandwidth _bandwidth;
uint8_t _srd;

__weak void MPU9250_OnActivate()
{
}
#ifndef USE_SPI
bool	MPU9250_IsConnected()
{
	if(HAL_I2C_IsDeviceReady(&_MPU9250_I2C,_dev_add,1,HAL_MAX_DELAY)==HAL_OK)
		return true;
	else
		return false;	
}

static void MPU_I2C_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	HAL_I2C_Mem_Write(&_MPU9250_I2C,_dev_add,WriteAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumByteToWrite,HAL_MAX_DELAY);
}

static void MPU_I2C_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_I2C_Master_Transmit(&_MPU9250_I2C,_dev_add,&data,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&_MPU9250_I2C,_dev_add,pBuffer,NumByteToRead,HAL_MAX_DELAY);
}
#endif
#ifdef USE_SPI
static inline void MPU9250_Activate()
{
	MPU9250_OnActivate();
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

static uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

static void MPU_SPI_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MPU9250_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU9250_Deactivate();
}

static void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MPU9250_Activate();
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	MPU9250_Deactivate();
}
#endif
/* writes a byte to MPU9250 register given a register address and data */
static void writeRegister(uint8_t subAddress, uint8_t data)
{
#ifdef USE_SPI
	MPU_SPI_Write(&data, subAddress, 1);
#else
	MPU_I2C_Write(&data, subAddress, 1);
#endif
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
static void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
#ifdef USE_SPI
	MPU_SPI_Read(dest, subAddress, count);
#else
	MPU_I2C_Read(dest, subAddress, count);
#endif
}

/* writes a register to the AK8963 given a register address and data */
static void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// store the data for write
	writeRegister(I2C_SLV0_DO,data);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
static void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress);

	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(){
	// read the WHO AM I register
	readRegisters(WHO_AM_I,1,_buffer);
	// return the register value
	return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963(){
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I,1,_buffer);
	// return the register value
	return _buffer[0];
}


static void MPU9250_ReadCalibMAG()
{
	// read the AK8963 ASA registers and compute magnetometer scale factors
	uint8_t magData[3];
	readAK8963Registers(AK8963_ASA, 3, magData);
	for(int i = 0; i < 3; i++) {
		_mag_adjust[i] = ((((float)magData[i]) - 128.0f)/(256.0f) + 1.0f) * Magnetometer_Sensitivity_Scale_Factor; // micro Tesla
	}
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
#ifndef USE_SPI
	while(MPU9250_IsConnected() == false) {
		HAL_Delay(10);
	}
#endif
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);
	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN);
	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMNT_1,PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI();
	if((who != 0x71) && ( who != 0x73)) {
		return 1;
	}

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2,SEN_ENABLE);

	// setting accel range to 8G as default
	MPU9250_SetAccelRange(ACCEL_RANGE_8G);

	// setting the gyro range to 250DPS as default
	MPU9250_SetGyroRange(GYRO_RANGE_250DPS);

	// setting bandwidth to 184Hz as default
	MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_92HZ);

	// setting the sample rate divider to 0 as default
	writeRegister(SMPDIV,0x00);

	// enable I2C master mode
	writeRegister(USER_CTRL,I2C_MST_EN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL,I2C_MST_CLK);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 0x48 ) {
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	//readAK8963Registers(AK8963_ASA, 3, _mag_adjust);
	MPU9250_ReadCalibMAG();

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL,7,_buffer);

	// successful init, return 0
	return 0;
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
	switch (range){
	case ACCEL_RANGE_2G:
		writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G);
		_accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
		break;

	case ACCEL_RANGE_4G:
		writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G);
		_accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
		break;

	case ACCEL_RANGE_8G:
		writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G);
		_accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
		break;

	case ACCEL_RANGE_16G:
		writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G);
		_accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
		break;
	}
	_accelRange = range;
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
	switch (range) {
	case GYRO_RANGE_250DPS:
		writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS);
		_gyroScale = 250.0f/32767.5f; // setting the gyro scale to 250DPS
		break;

	case GYRO_RANGE_500DPS:
		writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS);
		_gyroScale = 500.0f/32767.5f; // setting the gyro scale to 500DPS
		break;

	case GYRO_RANGE_1000DPS:
		writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS);
		_gyroScale = 1000.0f/32767.5f; // setting the gyro scale to 1000DPS
		break;

	case GYRO_RANGE_2000DPS:
		writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS);
		_gyroScale = 2000.0f/32767.5f; // setting the gyro scale to 2000DPS
		break;
	}
	_gyroRange = range;
}



/* sets the DLPF bandwidth to values other than default */
void MPU9250_setDlpfBandwidth(DlpfBandwidth bandwidth)
{
	switch(bandwidth) {
	case DLPF_BANDWIDTH_184HZ: {
		writeRegister(ACCEL_CONFIG2, DLPF_184); // setting accel bandwidth to 184Hz
		writeRegister(CONFIG, DLPF_184); // setting gyro bandwidth to 184Hz
		break;
	}

	case DLPF_BANDWIDTH_92HZ: {
		writeRegister(ACCEL_CONFIG2,DLPF_92); // setting accel bandwidth to 92Hz
		writeRegister(CONFIG,DLPF_92); // setting gyro bandwidth to 92Hz
		break;
	}

	case DLPF_BANDWIDTH_41HZ: {
		writeRegister(ACCEL_CONFIG2,DLPF_41); // setting accel bandwidth to 41Hz
		writeRegister(CONFIG,DLPF_41); // setting gyro bandwidth to 41Hz
		break;
	}

	case DLPF_BANDWIDTH_20HZ: {
		writeRegister(ACCEL_CONFIG2,DLPF_20); // setting accel bandwidth to 20Hz
		writeRegister(CONFIG,DLPF_20); // setting gyro bandwidth to 20Hz
		break;
	}

	case DLPF_BANDWIDTH_10HZ: {
		writeRegister(ACCEL_CONFIG2,DLPF_10); // setting accel bandwidth to 10Hz
		writeRegister(CONFIG,DLPF_10); // setting gyro bandwidth to 10Hz
		break;
	}

	case DLPF_BANDWIDTH_5HZ: {
		writeRegister(ACCEL_CONFIG2,DLPF_5); // setting accel bandwidth to 5Hz
		writeRegister(CONFIG,DLPF_5); // setting gyro bandwidth to 5Hz
		break;
	}

	}

	_bandwidth = bandwidth;
}

/* sets the sample rate divider to values other than default */
void MPU9250_setSrd(uint8_t srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(SMPDIV,19);

	if(srd > 9) {
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL,7,_buffer);

	} else {
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL,7,_buffer);
	}

	writeRegister(SMPDIV, srd);
	_srd = srd;
}

/* enables the data ready interrupt */
void MPU9250_enableDataReadyInterrupt()
{
	/* setting the interrupt */
	writeRegister(INT_PIN_CFG,INT_PULSE_50US); // setup interrupt, 50 us pulse
	writeRegister(INT_ENABLE,INT_RAW_RDY_EN); // set to data ready
}

/* disables the data ready interrupt */
void MPU9250_disableDataReadyInterrupt()
{
	writeRegister(INT_ENABLE,INT_DISABLE); // disable interrupt
}

/* configures and enables wake on motion, low power mode */
void MPU9250_enableWakeOnMotion(float womThresh_mg, LpAccelOdr odr)
{
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(PWR_MGMNT_1,PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	writeRegister(PWR_MGMNT_1, 0x00); // cycle 0, sleep 0, standby 0
	writeRegister(PWR_MGMNT_2,DIS_GYRO); // disable gyro measurements

	writeRegister(ACCEL_CONFIG2,DLPF_184); // setting accel bandwidth to 184Hz

	writeRegister(INT_ENABLE,INT_WOM_EN); // enabling interrupt to wake on motion
	writeRegister(MOT_DETECT_CTRL,(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)); // enabling accel hardware intelligence

	float _womThreshold = womThresh_mg * 0.25f;
	writeRegister(WOM_THR,_womThreshold); // setting wake on motion threshold
	writeRegister(LP_ACCEL_ODR,(uint8_t)odr); // set frequency of wakeup
	writeRegister(PWR_MGMNT_1,PWR_CYCLE); // switch to accel low power mode
}

/* read the data, each argiment should point to a array for x, y, and z , at last read the temperature data */
void MPU9250_GetData(float* AccData, float* MagData, float* GyroData, float *TempData)
{
	// grab the data from the MPU9250
	readRegisters(ACCEL_OUT, 21, _buffer);

	// combine into 16 bit values
	const int16_t _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	const int16_t _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	const int16_t _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
	const int16_t _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
	const int16_t _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	const int16_t _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	const int16_t _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
	const int16_t _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
	const int16_t _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
	const int16_t _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];

	// combine into 16 bit values
	AccData[0] = ((float)(_axcounts * _accelScale) - _abias[0]) * _amultiply[0];
	AccData[1] = ((float)(_aycounts * _accelScale) - _abias[1]) * _amultiply[1];
	AccData[2] = ((float)(_azcounts * _accelScale) - _abias[2]) * _amultiply[2];

	GyroData[0] = (float)(_gxcounts * _gyroScale) - _gbias[0];
	GyroData[1] = (float)(_gycounts * _gyroScale) - _gbias[1];
	GyroData[2] = (float)(_gzcounts * _gyroScale) - _gbias[2];

	MagData[0] = (float)((float)_hxcounts * _mag_adjust[0]);
	MagData[1] = (float)((float)_hycounts * _mag_adjust[1]);
	MagData[2] = (float)((float)_hzcounts * _mag_adjust[2]);

	//transform Temperature data into float 32 bit values 
	*TempData = ((((float) _tcounts) - RoomTemp_Offset)/Temp_Sensitivity) + RoomTemp_Offset; //according to the register map file 4.23, TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/ Temp_Sensitivity) + 21degC
	//according to the datasheet 3.4.2, RoomTemp_Offset = 21, Sensitivity = 333.87
}


/* estimates the gyro biases */
void MPU9250_calibrateGyro()
{
	// save  configuration
	//const AccelRange _accelRange_saved = _accelRange;
	const GyroRange _gyroRange_saved =  _gyroRange;
	const DlpfBandwidth _bandwidth_saved = _bandwidth;
	const uint8_t _srd_saved = _srd;

	// set the range, bandwidth, and srd
	MPU9250_SetGyroRange(GYRO_RANGE_250DPS);
	MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ);
	MPU9250_setSrd(19);

	// take samples and find bias
	double _gxbD = 0;
	double _gybD = 0;
	double _gzbD = 0;

	// main values
	float tmpAccData[3];
	float tmpMagData[3];
	float tmpGyroData[3];
	float tmpTempData;

	for (size_t i=0; i < CALIBRATE_CNT; i++) {
		MPU9250_GetData(tmpAccData, tmpMagData, tmpGyroData, &tmpTempData);
		_gxbD += (tmpGyroData[0] + _gbias[0])/((double) CALIBRATE_CNT);
		_gybD += (tmpGyroData[1] + _gbias[1])/((double) CALIBRATE_CNT);
		_gzbD += (tmpGyroData[2] + _gbias[2])/((double) CALIBRATE_CNT);
		HAL_Delay(20);
	}
	_gbias[0] = (float)_gxbD;
	_gbias[1] = (float)_gybD;
	_gbias[2] = (float)_gzbD;

	// set the range, bandwidth, and srd back to what they were
	MPU9250_SetGyroRange(_gyroRange_saved);
	MPU9250_setDlpfBandwidth(_bandwidth_saved);
	MPU9250_setSrd(_srd_saved);
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
void MPU9250_calibrateAccel()
{
	// save  configuration
	const AccelRange _accelRange_saved = _accelRange;
	//const GyroRange _gyroRange_saved =  _gyroRange;
	const DlpfBandwidth _bandwidth_saved = _bandwidth;
	const uint8_t _srd_saved = _srd;

	// set the range, bandwidth, and srd
	MPU9250_SetAccelRange(ACCEL_RANGE_2G);
	MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_20HZ);
	MPU9250_setSrd(19);

	// take samples and find min / max
	double _axbD = 0;
	double _aybD = 0;
	double _azbD = 0;

	// main values
	float tmpAccData[3];
	float tmpMagData[3];
	float tmpGyroData[3];
	float tmpTempData;


	for (size_t i=0; i < CALIBRATE_CNT; i++) {
		MPU9250_GetData(tmpAccData, tmpMagData, tmpGyroData, &tmpTempData);
		_axbD += (tmpAccData[0]/_amultiply[0] + _abias[0])/((double)CALIBRATE_CNT);
		_aybD += (tmpAccData[1]/_amultiply[1] + _abias[1])/((double)CALIBRATE_CNT);
		_azbD += (tmpAccData[2]/_amultiply[2] + _abias[2])/((double)CALIBRATE_CNT);
		HAL_Delay(20);
	}
	if (_axbD > 9.0f) {
		_axmax = (float)_axbD;
	}
	if (_aybD > 9.0f) {
		_aymax = (float)_aybD;
	}
	if (_azbD > 9.0f) {
		_azmax = (float)_azbD;
	}
	if (_axbD < -9.0f) {
		_axmin = (float)_axbD;
	}
	if (_aybD < -9.0f) {
		_aymin = (float)_aybD;
	}
	if (_azbD < -9.0f) {
		_azmin = (float)_azbD;
	}

	// find bias and scale factor
	if ((fabs(_axmin) > 9.0f) && (fabs(_axmax) > 9.0f)) {
		_abias[0] = (_axmin + _axmax) / 2.0f;
		_amultiply[0] = G/((fabs(_axmin) + fabs(_axmax)) / 2.0f);
	}
	if ((fabs(_aymin) > 9.0f) && (fabs(_aymax) > 9.0f)) {
		_abias[1] = (_aymin + _aymax) / 2.0f;
		_amultiply[1] = G/((fabs(_aymin) + fabs(_aymax)) / 2.0f);
	}
	if ((fabs(_azmin) > 9.0f) && (fabs(_azmax) > 9.0f)) {
		_abias[2] = (_azmin + _azmax) / 2.0f;
		_amultiply[2] = G/((fabs(_azmin) + fabs(_azmax)) / 2.0f);
	}

	// set the range, bandwidth, and srd back to what they were
	MPU9250_SetAccelRange(_accelRange_saved);
	MPU9250_setDlpfBandwidth(_bandwidth_saved);
	MPU9250_setSrd(_srd_saved);
}

void MPU9250_getCalibrations(float *_gyroBiases, float *_accBiases, float *_accMultiplications)
{
	for(int i = 0; i < 3; ++i) {
		_gyroBiases[i] = _gbias[i];
		_accBiases[i] = _abias[i];
		_accMultiplications[i] = _amultiply[i];
	}
}

void MPU9250_setCalibrations(float *_gyroBiases, float *_accBiases, float *_accMultiplications)
{
	for(int i = 0; i < 3; ++i) {
		_gbias[i] = _gyroBiases[i];
		_abias[i] = _accBiases[i];
		_amultiply[i] = _accMultiplications[i];
	}
}


/* reads data from the MPU9250 FIFO and stores in buffer ------------------------------------------------------------------------------------- */

/* configures and enables the FIFO buffer  */
void MPU9250FIFO_enableFifo(bool accel,bool gyro,bool mag,bool temp)
{
	writeRegister(USER_CTRL, (0x40 | I2C_MST_EN));
	writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(mag*FIFO_MAG)|(temp*FIFO_TEMP));

	_enFifoAccel = accel;
	_enFifoGyro = gyro;
	_enFifoMag = mag;
	_enFifoTemp = temp;
	_fifoFrameSize = accel*6 + gyro*6 + mag*7 + temp*2;
}


void MPU9250FIFO_readFifo()
{
	// get the fifo size
	readRegisters(FIFO_COUNT, 2, _buffer);
	_fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
	// read and parse the buffer
	for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
		// grab the data from the MPU9250
		readRegisters(FIFO_READ,_fifoFrameSize,_buffer);

		if (_enFifoAccel) {
			// combine into 16 bit values
			int16_t _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
			int16_t _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
			int16_t _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
			// transform and convert to float values
			_axFifo[i] = (((float)(_axcounts) * _accelScale) - _abias[0]) * _amultiply[0];
			_ayFifo[i] = (((float)(_aycounts) * _accelScale) - _abias[1]) * _amultiply[1];
			_azFifo[i] = (((float)(_azcounts) * _accelScale) - _abias[2]) * _amultiply[2];
			_aSize = _fifoSize/_fifoFrameSize;
		}
		if (_enFifoTemp) {
			// combine into 16 bit values
			int16_t _tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
			// transform and convert to float values
			_tFifo[i] = ((((float) _tcounts) - RoomTemp_Offset)/Temp_Sensitivity) + RoomTemp_Offset;
			_tSize = _fifoSize/_fifoFrameSize;
		}
		if (_enFifoGyro) {
			// combine into 16 bit values
			int16_t _gxcounts = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
			int16_t _gycounts = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
			int16_t _gzcounts = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
			// transform and convert to float values
			_gxFifo[i] = ((float)(_gxcounts) * _gyroScale) - _gbias[0];
			_gyFifo[i] = ((float)(_gycounts) * _gyroScale) - _gbias[1];
			_gzFifo[i] = ((float)(_gzcounts) * _gyroScale) - _gbias[2];
			_gSize = _fifoSize/_fifoFrameSize;
		}
		if (_enFifoMag) {
			// combine into 16 bit values
			int16_t _hxcounts = (((int16_t)_buffer[1 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[0 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
			int16_t _hycounts = (((int16_t)_buffer[3 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[2 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
			int16_t _hzcounts = (((int16_t)_buffer[5 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[4 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
			// transform and convert to float values
			_hxFifo[i] = (((float)(_hxcounts) * _mag_adjust[0]));
			_hyFifo[i] = (((float)(_hycounts) * _mag_adjust[1]));
			_hzFifo[i] = (((float)(_hzcounts) * _mag_adjust[2]));
			_hSize = _fifoSize/_fifoFrameSize;
		}
	}
}


/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU9250FIFO_getFifoAccelX_mss(size_t *size,float* data) {
	*size = _aSize;
	memcpy(data,_axFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU9250FIFO_getFifoAccelY_mss(size_t *size,float* data) {
	*size = _aSize;
	memcpy(data,_ayFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU9250FIFO_getFifoAccelZ_mss(size_t *size,float* data) {
	*size = _aSize;
	memcpy(data,_azFifo,_aSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, deg/s */
void MPU9250FIFO_getFifoGyroX_rads(size_t *size,float* data) {
	*size = _gSize;
	memcpy(data,_gxFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, deg/s */
void MPU9250FIFO_getFifoGyroY_rads(size_t *size,float* data) {
	*size = _gSize;
	memcpy(data,_gyFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, deg/s */
void MPU9250FIFO_getFifoGyroZ_rads(size_t *size,float* data) {
	*size = _gSize;
	memcpy(data,_gzFifo,_gSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the x direction, uT */
void MPU9250FIFO_getFifoMagX_uT(size_t *size,float* data) {
	*size = _hSize;
	memcpy(data,_hxFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the y direction, uT */
void MPU9250FIFO_getFifoMagY_uT(size_t *size,float* data) {
	*size = _hSize;
	memcpy(data,_hyFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the z direction, uT */
void MPU9250FIFO_getFifoMagZ_uT(size_t *size,float* data) {
	*size = _hSize;
	memcpy(data,_hzFifo,_hSize*sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void MPU9250FIFO_getFifoTemperature_C(size_t *size,float* data) {
	*size = _tSize;
	memcpy(data,_tFifo,_tSize*sizeof(float));
}

