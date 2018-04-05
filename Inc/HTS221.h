#ifndef HTS221_H
#define HTS221_H

#include "stm32f7xx_hal.h"

#define WHO_AM_I 0x0F
#define AV_CONF 0x10
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define STATUS_REG 0x27
#define HUMIDITY_OUT_L 0x28
#define HUBIDITY_OUT_H 0x29
#define TEMP_OUT_L 0x2A
#define TEMP_OUT_H 0x2B
#define HTS221_H0_RH_X2 0x30
#define HTS221_H1_RH_X2 0x31
#define HTS221_T0_DEGC_X8 0x32
#define HTS221_T1_DEGC_X8 0x33
#define HTS221_T0_T1_DEGC_H2 0x35
#define HTS221_H0_T0_OUT_L 0x36
#define HTS221_H0_T0_OUT_H 0x37
#define HTS221_H1_T0_OUT_L 0x3A
#define HTS221_H1_T0_OUT_H 0x3B
#define HTS221_T0_OUT_L 0x3C
#define HTS221_T0_OUT_H 0x3D
#define HTS221_T1_OUT_L 0x3E
#define HTS221_T1_OUT_H 0x3F

#define HTS221_READ_ADDRESS 0xBF
#define HTS221_WRITE_ADDRESS 0xBF // 0xBE

typedef struct tempSensor{
	float humidity;
	float temperature;
}tempSensor;

tempSensor tempData;

void HTS221_Get_Humidity();
void HTS221_Get_Temperature();
void transmitCommandLRF();
uint8_t configHTS221(I2C_HandleTypeDef * hi2c);
void getCalibrationCoefs();
void HTS221_write(uint8_t regAdress, uint8_t data);
void HTS221_read(uint8_t regAdress, uint8_t * data, uint8_t bytesCount);

#endif
