/*
 * MPU9250.c
 *
 *  Created on: Mar 29, 2018
 *      Author: magratey
 */
#include "stm32f7xx_hal.h"
#include "math.h"
#include "MPU9250.h"
#include "gpio.h"

SPI_HandleTypeDef * hspi;

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate

// parameters for 6 DoF sensor fusion calculations
float PI = 3.14159265358979323846f;
float GyroMeasError;// = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta;// = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift;// = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta;// = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 0.5f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.01f

// imuStruct imuData;
//float pitch, yaw, roll;
float deltat = 0.0f;                             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method

uint32_t MPU9250_lastUpdate, MPU9250_firstUpdate, MPU9250_now;// used to calculate integration interval
float MPU9250_deltat;// = 0.0f;                              // integration interval for both filter schemes

int16_t ax_raw,ay_raw,az_raw,
        gx_raw,gy_raw,gz_raw,
		mx_raw,my_raw,mz_raw;
float ax,ay,az,gx,gy,gz,mx,my,mz;

//===================================================================================================================
//====== Set of useful function to access acceleratio, gyroscope, and temperature data
//===================================================================================================================

uint8_t writeReg( uint8_t writeAddr, uint8_t writeData )
{
	uint8_t address = 0;
	uint8_t data = 0;

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	address = writeAddr;
	HAL_SPI_TransmitReceive(hspi, &address, &data, 1, 0x1000);
	address = writeData;
	HAL_SPI_TransmitReceive(hspi, &address, &data, 1, 0x1000);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

	return data;
}

uint8_t readReg( uint8_t readAddr)
{
	return writeReg( readAddr | READ_FLAG, 0x00);
}

void readRegs( uint8_t readAddr, uint8_t count, uint8_t * dest)
{
	unsigned int  i = 0;

	uint8_t address = 0;
	uint8_t data = 0;

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	address = readAddr | READ_FLAG;
	HAL_SPI_TransmitReceive(hspi, &address, &data, 1, 0x1000);

	address = 0x00;
	for(i=0; i<count; i++)
	    	HAL_SPI_TransmitReceive(hspi, &address, &dest[i], 1, 0x1000);

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

}

uint8_t readMagReg(uint8_t subAddr)
{
  writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS|0b10000000);
  writeReg(I2C_SLV0_REG, subAddr);
  writeReg(I2C_SLV0_CTRL, 0x81);
  HAL_Delay(1);
  uint8_t rb = readReg(EXT_SENS_DATA_00);

  return rb;
}

void readMagRegs(uint8_t subAddr, uint8_t count, uint8_t *dest)
{
  int i = 0;
  for(; i < count; ++i){
    writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS|0b10000000);
    writeReg(I2C_SLV0_REG, subAddr+i);
    writeReg(I2C_SLV0_CTRL,0x81);
    HAL_Delay(1);
    dest[i] = readReg(EXT_SENS_DATA_00);

  }

}

void writeMagReg(uint8_t addr,uint8_t data){
    writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS&0b01111111);
    writeReg(I2C_SLV0_REG, addr);
    writeReg(I2C_SLV0_DO, data);
    writeReg(I2C_SLV0_CTRL, 0x81);
    HAL_Delay(1);
}


uint8_t initMPU(SPI_HandleTypeDef * spi)
{
	hspi = spi;

	GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
	GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	resetMPU9250(); // Reset registers to default in preparation for device calibration
	HAL_Delay(200);
	calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	HAL_Delay(200);
	initMPU9250();
	initAK8963(magCalibration);

	getAres(); // Get accelerometer sensitivity
	getGres(); // Get gyro sensitivity
	getMres(); // Get magnetometer sensitivity
	magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
	magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

	return readReg(WHO_AM_I_MPU9250);
}

void getMres()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.0*4219.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.0*4219.0/32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}


void getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void MPU9250_getAcceleration(int16_t * ax_raw, int16_t * ay_raw, int16_t * az_raw)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readRegs(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  * ax_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  * ay_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  * az_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void MPU9250_getRotation(int16_t * gx_raw, int16_t * gy_raw, int16_t * gz_raw)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readRegs(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  * gx_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  * gy_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  * gz_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void MPU9250_getCompass(int16_t * mx_raw, int16_t * my_raw, int16_t * mz_raw)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readMagReg(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to t
    readMagRegs(AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then   report data
      * mx_raw = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
      * my_raw = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
     * mz_raw = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readRegs(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
}


void resetMPU9250()
{
  // reset device
  writeReg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);
}

void initAK8963(float *destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  //writeReg(AK8963_I2CDIS,0b00011011);
  //writeReg(I2C_SLV4_ADDR,AK8963_ADDRESS);
 // writeReg(I2C_MST_CTRL,I2C_MST_CTRL|0x0D);
  writeMagReg(AK8963_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(10);
  writeMagReg(AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(10);
  readMagRegs(AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
  writeMagReg(AK8963_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeMagReg(AK8963_CNTL1, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  HAL_Delay(10);
}


void initMPU9250()
{
	uint8_t tries;
    for (tries = 0; tries<5; tries++){
	    writeReg(PWR_MGMT_1,0x80);
        HAL_Delay(100);
    }

	writeReg(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	writeReg(USER_CTRL, 0x10); // Disable I2C
	HAL_Delay(100);
	// get stable time source
	writeReg(PWR_MGMT_1, 0x01); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	HAL_Delay(200);
	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeReg(CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeReg(SMPLRT_DIV, 0x00); // Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readReg(GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeReg(GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readReg(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	writeReg(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readReg(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeReg(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeReg(INT_PIN_CFG, 0x22);
	writeReg(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	HAL_Delay(100);

	writeReg(USER_CTRL, 0x20);
	writeReg(I2C_MST_CTRL, 0x09);
	HAL_Delay(100);

	writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS);

	writeReg(I2C_SLV0_REG, AK8963_CNTL2);
	writeReg(I2C_SLV0_DO, 0x01);
	writeReg(I2C_SLV0_CTRL, 0x81);
	HAL_Delay(200);

	writeReg(I2C_SLV0_REG, AK8963_CNTL1);
	writeReg(I2C_SLV0_DO, 0x12);
	writeReg(I2C_SLV0_CTRL, 0x81);
	HAL_Delay(100);


}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeReg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeReg(PWR_MGMT_1, 0x01);
  writeReg(PWR_MGMT_2, 0x00);
  HAL_Delay(200);

// Configure device for bias calculation
  writeReg(INT_ENABLE, 0x00);   // Disable all interrupts
  writeReg(FIFO_EN, 0x00);      // Disable FIFO
  writeReg(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeReg(I2C_MST_CTRL, 0x00); // Disable I2C master
  writeReg(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeReg(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(15);

// Configure MPU9250 gyro and accelerometer for bias calculation
  writeReg(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeReg(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeReg(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeReg(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeReg(USER_CTRL, 0x40);   // Enable FIFO
  writeReg(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeReg(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readRegs(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readRegs(FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeReg(XG_OFFSET_H, data[0]);
  writeReg(XG_OFFSET_L, data[1]);
  writeReg(YG_OFFSET_H, data[2]);
  writeReg(YG_OFFSET_L, data[3]);
  writeReg(ZG_OFFSET_H, data[4]);
  writeReg(ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readRegs(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readRegs(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readRegs(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeReg(XA_OFFSET_H, data[0]);
  writeReg(XA_OFFSET_L, data[1]);
  writeReg(YA_OFFSET_H, data[2]);
  writeReg(YA_OFFSET_L, data[3]);
  writeReg(ZA_OFFSET_H, data[4]);
  writeReg(ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;

   int ii = 0;

  writeReg(SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
  writeReg(CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeReg(GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
  writeReg(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeReg(ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for(ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

  readRegs(ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readRegs(GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   writeReg(ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeReg(GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   HAL_Delay(25); // Delay a while to let the device stabilize

  for(ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

  readRegs(ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readRegs(GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
   writeReg(ACCEL_CONFIG, 0x00);
   writeReg(GYRO_CONFIG, 0x00);
   HAL_Delay(25); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readReg(SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readReg(SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readReg(SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readReg(SELF_TEST_X_GYRO); // X-axis gyro self-test results
   selfTest[4] = readReg(SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
   selfTest[5] = readReg(SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (ii = 0; ii < 3; ii++) {
     destination[ii] = 100.0*((float)(aSTAvg[ii] - aAvg[ii]))/factoryTrim[ii]; // Report percent differences
     destination[ii+3] = 100.0*((float)(gSTAvg[ii] - gAvg[ii]))/factoryTrim[ii+3]; // Report percent differences
   }

}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MPU9250_MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	float _8bx = 2.0f * _4bx;
	float _8bz = 2.0f * _4bz;

	// Gradient decent algorithm corrective step
	s1= -_2q3*(2*(q2q4 - q1q3) - ax) + _2q2*(2*(q1q2 + q3q4) - ay) + -_4bz*q3*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx) + (-_4bx*q4+_4bz*q2)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my) + _4bx*q3*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s2= _2q4*(2*(q2q4 - q1q3) - ax) + _2q1*(2*(q1q2 + q3q4) - ay) + -4*q2*(2*(0.5 - q2q2 - q3q3) - az) + _4bz*q4*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx) + (_4bx*q3+_4bz*q1)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my) + (_4bx*q4-_8bz*q2)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s3= -_2q1*(2*(q2q4 - q1q3) - ax) + _2q4*(2*(q1q2 + q3q4) - ay) + (-4*q3)*(2*(0.5 - q2q2 - q3q3) - az) + (-_8bx*q3-_4bz*q1)*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx)+(_4bx*q2+_4bz*q4)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my)+(_4bx*q1-_8bz*q3)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s4= _2q2*(2*(q2q4 - q1q3) - ax) + _2q3*(2*(q1q2 + q3q4) - ay)+(-_8bx*q4+_4bz*q2)*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx)+(-_4bx*q1+_4bz*q3)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my)+(_4bx*q2)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * MPU9250_deltat;
	q2 += qDot2 * MPU9250_deltat;
	q3 += qDot3 * MPU9250_deltat;
	q4 += qDot4 * MPU9250_deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objective funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx = 0.0f, gbiasy = 0.0f, gbiasz = 0.0f;  // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  //            float _2q1q3 = 2.0f * q1 * q3;
  //            float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * MPU9250_deltat * zeta;
  gbiasy += gerry * MPU9250_deltat * zeta;
  gbiasz += gerrz * MPU9250_deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * MPU9250_deltat;
  q2 += (qDot2 -(beta * hatDot2)) * MPU9250_deltat;
  q3 += (qDot3 -(beta * hatDot3)) * MPU9250_deltat;
  q4 += (qDot4 -(beta * hatDot4)) * MPU9250_deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

void MPU9250_TakeAndCalcData()
{
  int i;

  //MPU9250_getMotion6(&ax_raw,&ay_raw,&az_raw,&gx_raw,&gy_raw,&gz_raw);
  MPU9250_getAcceleration(&ax_raw,&ay_raw,&az_raw);
  MPU9250_getRotation(&gx_raw,&gy_raw,&gz_raw);
//  MPU9250_getCompass(&mx_raw,&my_raw,&mz_raw);

  // Now we'll calculate the accleration value into actual g's
  ax = (float)ax_raw * aRes;  // get actual g value, this depends on scale being set
  ay = (float)ay_raw * aRes;
  az = (float)az_raw * aRes;

  // Calculate the gyro value into actual degrees per second
  gx = (float)gx_raw * gRes;  // get actual gyro value, this depends on scale being set
  gy = (float)gy_raw * gRes;
  gz = (float)gz_raw * gRes;

  // Calculate the compass value into actual degrees per second
//  mx = (float)mx_raw * mRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
//  my = (float)my_raw * mRes; // - gyroBias[1];
//  mz = (float)mz_raw * mRes; // - gyroBias[2];

  //debug_take();

  for (i = 1; i <= 1; i++)
  {
    MPU9250_now = HAL_GetTick();
    MPU9250_deltat = (float)((MPU9250_now - MPU9250_lastUpdate)/1000.0f) ; // set integration time by time elapsed since last filter update
    MPU9250_lastUpdate = MPU9250_now;

    // Pass gyro rate as rad/s
   // MPU9250_MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz );
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);
   // MPU9250_MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz );

  }

  if (MPU9250_lastUpdate - MPU9250_firstUpdate > 10000.0f) {
    //beta = 0.04;  // decrease filter gain after stabilized
    //zeta = 0.015; // increasey bias drift gain after stabilized
    beta = 0.09;  // decrease filter gain after stabilized
    zeta = 0.005; // increasey bias drift gain after stabilized
  }

}


void MPU9250_CalcYPR()
{
  imuData.yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  imuData.pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  imuData.roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  imuData.pitch *= 180.0f / PI;
  imuData.yaw   *= 180.0f / PI;
  imuData.roll  *= 180.0f / PI;

 // debug_calc();
}

imuStruct  imuGetData(){
	return imuData;
}


