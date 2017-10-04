/*
 * utils.c
 *
 *  Created on: 2 de oct. de 2017
 *      Author: cubecat
 */

#include "utils.h"
#include "deploy_execution.h"
#include <string.h>

volatile static uint8_t _flag_adc_temp_done = 0;

static deploy_command_t _i2c_cmd;

static TSCALIB_t calib_data;

static struct {
	float internal;
	float external;
}deploy_temperatures;

static void get_tscalib(TSCALIB_t *data)
{
	const volatile uint16_t *cal_temp_1;
	const volatile uint16_t *cal_temp_2;
	const volatile uint16_t *cal_vref;
	cal_temp_1 	= (const volatile uint16_t *) HK_TEMP_CAL_REG_1;
	cal_temp_2 	= (const volatile uint16_t *) HK_TEMP_CAL_REG_2;
	cal_vref 	= (const volatile uint16_t *) HK_VREF_CAL_REG;
	data->TS_CAL_1	= *cal_temp_1;
	data->TS_CAL_2 	= *cal_temp_2;
	data->VREF 		= *cal_vref;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
		case FEEDBACK_1_Pin:
		case FEEDBACK_2_Pin:
		case FEEDBACK_3_Pin:
		case FEEDBACK_4_Pin:
			unset_deploy();
			break;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	osSignalSet(controlTaskHandle, I2C_WAIT_MASTER);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	osSignalSet(controlTaskHandle, I2C_WAIT_MASTER);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	osSignalSet(i2cTaskHandle, I2C_WAIT_SLAVE);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	osSignalSet(i2cTaskHandle, I2C_WAIT_SLAVE);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C2)
		osSignalSet(i2cTaskHandle, I2C_WAIT_ERROR);
	else if (hi2c->Instance == I2C1)
		osSignalSet(controlTaskHandle, I2C_WAIT_ERROR);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	osSignalSet(controlTaskHandle, ADC_LECTURE_DONE);
}


/*
 * Transfer is composed by a Transmission and Reception of a I2C command
 * To-send i2c is at t pointer and sized tlen
 * To-receive i2c is at r pointer and sized rlen
 * In case rlen or tlen == 0, it will be a only-rx or only-rx i2c transaction
 * Error code is returned (0 == no error)
 */
int i2c_master_transfer(uint8_t _7b_address, void *t, size_t tlen, void *r, size_t rlen)
{
	if (tlen > 0) {
		if (HAL_I2C_Master_Transmit_DMA(&hi2c2, _7b_address<<1, (uint8_t *) t, tlen) != HAL_OK) {
			return -1;
		}else {
			osSignalWait(I2C_WAIT_MASTER | I2C_WAIT_ERROR, osWaitForever);
		}
	}
	/* no errors --> we reached here */
	if (rlen > 0) {
		if (HAL_I2C_Master_Receive_DMA(&hi2c2, _7b_address<<1, (uint8_t *) r, rlen) != HAL_OK) {
			return -1;
		}else {
			osSignalWait(I2C_WAIT_MASTER | I2C_WAIT_ERROR, osWaitForever);
		}
	}
	return 0;
}


void turn_i2c_rx()
{
	memset(&_i2c_cmd, 0, sizeof(_i2c_cmd));
	HAL_I2C_Slave_Receive_DMA(&hi2c1, (uint8_t *) &_i2c_cmd, sizeof(_i2c_cmd));
}

int i2c_slave_message()
{
	osEvent ev;
	ev = osSignalWait(I2C_WAIT_SLAVE | I2C_WAIT_ERROR, osWaitForever);
	/* if the vent is osEventSignal --> return 1/true */
	if (ev.status == osEventSignal) {
		if (ev.value.signals & I2C_WAIT_SLAVE) {
			return 1;
		}
	}
	return 0;
}

int get_i2c_message(void *p)
{
	memcpy(p, &_i2c_cmd, sizeof(_i2c_cmd));
	return 2;
}

void i2c_slave_transmit(void *p, size_t len)
{
	if (HAL_I2C_Slave_Transmit_DMA(&hi2c1, p, len) == HAL_OK) {
		osSignalWait(I2C_WAIT_SLAVE | I2C_WAIT_ERROR, osWaitForever);
	}
}

void initialize_temp_sensor()
{
	get_tscalib(&calib_data);
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
}


float convert_temp_u16_f(uint16_t temp)
{
    int8_t msb;
    uint8_t lsb;
    msb = (temp>>8)&0xFF;
    lsb = temp&0xFF;
    if (msb < 0) {
        return (float) (1.0*msb - 1.0/256.0*lsb);
    }else {
        return (float) (1.0*msb + 1.0/256.0*lsb);
    }
}

uint16_t convert_temp_f_u16(float temp)
{
    int msb = (int) temp;
    float decpart = temp - msb;
    if (decpart < 0.0) {
        decpart = decpart * -1.0;
    }
    int lsb = (int) (decpart * 256);
    return ((msb&0xFF) << 8) | (lsb&0xff);
}

void refresh_temperatures()
{
	uint16_t adc_buffer[2];
	int32_t temp_cal;
	int32_t temp_uncal;
	uint16_t ref_volt;
	float val;
	uint8_t i2c_buffer[2];
	_flag_adc_temp_done = 0;
	HAL_ADC_Start_DMA(&hadc, (uint32_t *) adc_buffer, 2);
	osSignalWait(ADC_LECTURE_DONE, osWaitForever);
	/* transfer is complete */
	ref_volt = HK_VREF_VOLT_REF*calib_data.VREF/adc_buffer[HK_BUFFER_VREF_POS];
	temp_uncal = adc_buffer[HK_BUFFER_TEMP_POS] * ref_volt/HK_VREF_VOLT_REF;
	temp_cal = ( (int32_t) temp_uncal - (int32_t) calib_data.TS_CAL_1 );
	temp_cal *= (int32_t) HK_TEMP_MEAS_DIFF;
	temp_cal /= (int32_t) (calib_data.TS_CAL_2 - calib_data.TS_CAL_1);
	temp_cal += HK_TEMP_MEAS_1;
	deploy_temperatures.internal = (float)(temp_cal*1.0);

	memset(i2c_buffer, 0, 2);
	i2c_master_transfer(0x49, i2c_buffer, 1, i2c_buffer, 2);
	/* convert */
	val = (int8_t) i2c_buffer[0];
	if (val < 0.0) {
		val -= (i2c_buffer[1]>>4)/16.0;
	}else {
		val += (i2c_buffer[1]>>4)/16.0;
	}
	deploy_temperatures.external = (val);
}

uint16_t get_internal_temperature()
{

	return convert_temp_f_u16(deploy_temperatures.internal);
}

uint16_t get_external_temperature()
{
	return convert_temp_f_u16(deploy_temperatures.external);
}
