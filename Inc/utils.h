/*
 * utils.h
 *
 *  Created on: 2 de oct. de 2017
 *      Author: cubecat
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "main.h"
#include "stm32l0xx_hal.h"
#include "cmsis_os.h"

#define HK_TEMP_CAL_REG_1	0x1FF8007A
#define HK_TEMP_CAL_REG_2	0x1FF8007E
#define HK_VREF_CAL_REG		0x1FF80078

#define HK_VREF_VOLT_REF	3000
#define HK_TEMP_MEAS_1		30
#define HK_TEMP_MEAS_2		110
#define HK_TEMP_MEAS_DIFF	(HK_TEMP_MEAS_2 - HK_TEMP_MEAS_1)

#define HK_BUFFER_TEMP_POS	1
#define HK_BUFFER_VREF_POS 	0

#define I2C_WAIT_MASTER		(1<<0)
#define I2C_WAIT_SLAVE		(1<<1)
#define I2C_WAIT_ERROR		(1<<2)
#define ADC_LECTURE_DONE	(1<<3)

#define SIGNAL_DEPLOY_1A 	(1<<4)
#define SIGNAL_DEPLOY_1B 	(1<<5)
#define SIGNAL_DEPLOY_2A 	(1<<6)
#define SIGNAL_DEPLOY_2B 	(1<<7)
#define SIGNAL_DEPLOY_3A 	(1<<8)
#define SIGNAL_DEPLOY_3B 	(1<<9)
#define SIGNAL_DEPLOY_4A 	(1<<10)
#define SIGNAL_DEPLOY_4B 	(1<<11)

#define SIGNAL_DEPLOY_O_1A 	(1<<12)
#define SIGNAL_DEPLOY_O_1B 	(1<<13)
#define SIGNAL_DEPLOY_O_2A 	(1<<14)
#define SIGNAL_DEPLOY_O_2B 	(1<<15)
#define SIGNAL_DEPLOY_O_3A 	(1<<16)
#define SIGNAL_DEPLOY_O_3B 	(1<<17)
#define SIGNAL_DEPLOY_O_4A 	(1<<18)
#define SIGNAL_DEPLOY_O_4B 	(1<<19)

#define SIGNAL_ABORT_DEPLOY (1<<20)

#define SIGNAL_OFFSET_NORMAL(x)		(x<<4)
#define SIGNAL_OFFSET_OVERRIDE(x)	(x<<12)

typedef struct
{
	uint16_t VREF;
	uint16_t TS_CAL_1;
	uint16_t TS_CAL_2;
}TSCALIB_t;

extern osThreadId i2cTaskHandle;
extern osThreadId controlTaskHandle;

extern 	TIM_HandleTypeDef htim2;
extern 	I2C_HandleTypeDef hi2c1;
extern 	I2C_HandleTypeDef hi2c2;
extern 	ADC_HandleTypeDef hadc;

void 	enter_sleep_and_wait(uint16_t ms_count);

void 	setI2cSlaveCallback();
int 	waitI2cSlaveCallback();

void 	setI2cMasterCallback();
int 	waitI2cMasterCallback();

void 	setTimerCallback();
void 	waitTimerCallback();

void 	turn_i2c_rx();
int 	i2c_slave_message();
int 	get_i2c_message(void *p);

void 	i2c_slave_transmit(void *p, size_t len);

void 	initialize_temp_sensor();

float 	convert_temp_u16_f(uint16_t temp);
uint16_t 	convert_temp_f_u16(float temp);

void 	refresh_temperatures();
uint16_t 	get_internal_temperature();
uint16_t 	get_external_temperature();

#endif /* UTILS_H_ */
