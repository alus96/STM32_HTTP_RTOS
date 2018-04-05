#include "HTS221.h"
#include "I2Cdev.h"

int16_t H0_T0_out, H1_T0_out, H_T_out;
int16_t H0_rh, H1_rh;
int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
int16_t T0_degC, T1_degC;

#define LRF_DATA_BUFFER_SIZE 100

uint8_t lrfRxIdx, lrfTxIdx;
uint8_t lrfDataBuffer[LRF_DATA_BUFFER_SIZE];
uint8_t lrfCommandBuffer[30];

I2C_HandleTypeDef * hts221_hi2c;

void getCalibrationCoefs(){
	uint8_t buffer[2];
	uint32_t tmp;

	HTS221_read(HTS221_T0_DEGC_X8, &buffer, 2);

	HTS221_read(HTS221_T0_T1_DEGC_H2 , &tmp,1);

	T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
	T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
	T0_degC = T0_degC_x8_u16>>3;
	T1_degC = T1_degC_x8_u16>>3;

	HTS221_read(HTS221_T0_OUT_L, &buffer,2);
	T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	
	HTS221_read(HTS221_T1_OUT_L, &buffer,2);
	T1_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	HTS221_read(HTS221_H0_RH_X2, &buffer,2);
	H0_rh = buffer[0]>>1;
	H1_rh = buffer[1]>>1;

	HTS221_read(HTS221_H0_T0_OUT_L, &buffer,2);
	H0_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	HTS221_read(HTS221_H1_T0_OUT_L, &buffer,2);
	H1_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

}

void HTS221_Get_Humidity()
{
	uint8_t buffer[2];
	int32_t tmp;
	float value;

	HTS221_read(HUMIDITY_OUT_L, &buffer, 2);
	H_T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	tmp = ((uint32_t)(H_T_out - H0_T0_out)) * ((uint32_t)(H1_rh - H0_rh));
	value = (float)((float)(tmp)/(H1_T0_out - H0_T0_out) + (float)H0_rh);

	if(value <= 100 && value >= 0){
		tempData.humidity = value;
	}
}

void HTS221_Get_Temperature(){
	float value;
	uint8_t buffer[4], tmp;
	int32_t tmp32;

	HTS221_read(TEMP_OUT_L, &buffer, 2);

	T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	tmp32 = ((uint32_t)(T_out - T0_out)) * ((uint32_t)(T1_degC - T0_degC));
	value = (float)((float)(tmp32) /(T1_out - T0_out) + (float)T0_degC);

	if(value < 100 && value > - 60){
		tempData.temperature = value;
	}

}


uint8_t configHTS221(I2C_HandleTypeDef * hi2c){
	hts221_hi2c = hi2c;
	I2Cdev_init(hts221_hi2c);

	// I2Cdev_writeByte(0xBE,0x20,0b10000010);
 	 // I2Cdev_writeByte(0xBE,0x10,0x1B);
	HTS221_write(CTRL_REG1, 0x86);
	HTS221_write(AV_CONF, 0x1B);

	uint8_t reg_data;
	HTS221_read(WHO_AM_I, &reg_data, 1);

	getCalibrationCoefs();

	return reg_data;
}


void HTS221_write(uint8_t regAdress, uint8_t data){
	I2Cdev_writeByte(HTS221_WRITE_ADDRESS,regAdress, data);
}

void HTS221_read(uint8_t regAdress, uint8_t * data, uint8_t bytesCount){
	if(bytesCount > 1){
		I2Cdev_readWord(HTS221_READ_ADDRESS,regAdress | 0x80, data, 1);
	}else{
		I2Cdev_readByte(HTS221_READ_ADDRESS, regAdress, data, 1);
	}

}

