/*
 * deploy_execution.c
 *
 *  Created on: 2 de oct. de 2017
 *      Author: cubecat
 */

#include "deploy_execution.h"
#include "utils.h"
#include <string.h>

static uint8_t deploy_timer = 10;
static uint8_t deploy_error = 0;

void set_deploy_error(deploy_num_e which, deploy_side_e side, uint8_t set_unset)
{
	taskENTER_CRITICAL();
	if (set_unset == DEPLOY_ERR) {
		if (side == SIDE_B) {
			deploy_error |= ((which&0x01)<<4);
		}else {
			deploy_error |= (which&0x01);
		}
	}else if (set_unset == DEPLOY_NO_ERR){
		if (side == SIDE_B) {
			deploy_error &= ~((which&0x01)<<4);
		}else {
			deploy_error &= ~(which&0x01);
		}
	}
	taskEXIT_CRITICAL();
}

uint8_t get_deploy_error()
{
	uint8_t val;
	taskENTER_CRITICAL();
	val = deploy_error;
	taskEXIT_CRITICAL();
	return val;
}

void set_deploy_timer(uint8_t timer)
{
	taskENTER_CRITICAL();
	deploy_timer = timer;
	taskEXIT_CRITICAL();
}

uint8_t get_deploy_timer(void)
{
	uint8_t val;
	taskENTER_CRITICAL();
	val = deploy_timer;
	taskEXIT_CRITICAL();
	return val;
}

void set_deploy(deploy_num_e which, deploy_side_e side, int deploy_override)
{
	switch (which) {
		case DEPLOY_1:
			if (deploy_override == DEPLOY_NOT_OVERRIDE) {
				if (HAL_GPIO_ReadPin(FEEDBACK_1_GPIO_Port, FEEDBACK_1_Pin) == DEPLOYED)
					break;
			}
			if (side == SIDE_A) {
				HAL_GPIO_WritePin(DEPLOY_1A_GPIO_Port, DEPLOY_1A_Pin, GPIO_PIN_SET);
			}else {
				HAL_GPIO_WritePin(DEPLOY_1B_GPIO_Port, DEPLOY_1B_Pin, GPIO_PIN_SET);
			}
			break;
		case DEPLOY_2:
			if (deploy_override == DEPLOY_NOT_OVERRIDE) {
				if (HAL_GPIO_ReadPin(FEEDBACK_2_GPIO_Port, FEEDBACK_2_Pin) == DEPLOYED)
					break;
			}
			if (side == SIDE_A) {
				HAL_GPIO_WritePin(DEPLOY_2A_GPIO_Port, DEPLOY_2A_Pin, GPIO_PIN_SET);
			}else {
				HAL_GPIO_WritePin(DEPLOY_2B_GPIO_Port, DEPLOY_2B_Pin, GPIO_PIN_SET);
			}
			break;
		case DEPLOY_3:
			if (deploy_override == DEPLOY_NOT_OVERRIDE) {
				if (HAL_GPIO_ReadPin(FEEDBACK_3_GPIO_Port, FEEDBACK_3_Pin) == DEPLOYED)
					break;
			}
			if (side == SIDE_A) {
				HAL_GPIO_WritePin(DEPLOY_3A_GPIO_Port, DEPLOY_3A_Pin, GPIO_PIN_SET);
			}else {
				HAL_GPIO_WritePin(DEPLOY_3B_GPIO_Port, DEPLOY_3B_Pin, GPIO_PIN_SET);
			}
			break;
		case DEPLOY_4:
			if (deploy_override == DEPLOY_NOT_OVERRIDE) {
				if (HAL_GPIO_ReadPin(FEEDBACK_4_GPIO_Port, FEEDBACK_4_Pin) == DEPLOYED)
					break;
			}
			if (side == SIDE_A) {
				HAL_GPIO_WritePin(DEPLOY_4A_GPIO_Port, DEPLOY_4A_Pin, GPIO_PIN_SET);
			}else {
				HAL_GPIO_WritePin(DEPLOY_4B_GPIO_Port, DEPLOY_4B_Pin, GPIO_PIN_SET);
			}
			break;
		default:
			break;
	}
}

void unset_deploy()
{
	HAL_GPIO_WritePin(DEPLOY_1A_GPIO_Port, DEPLOY_1A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_1B_GPIO_Port, DEPLOY_1B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_2A_GPIO_Port, DEPLOY_2A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_2B_GPIO_Port, DEPLOY_2B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_3A_GPIO_Port, DEPLOY_3A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_3B_GPIO_Port, DEPLOY_3B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_4A_GPIO_Port, DEPLOY_4A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEPLOY_4B_GPIO_Port, DEPLOY_4B_Pin, GPIO_PIN_RESET);
}

uint8_t get_feedbacks()
{
	uint8_t feedback_signals[5];
	uint8_t ret = 0;
	int i;
	memset(feedback_signals, 0, sizeof(feedback_signals));
	feedback_signals[0] = HAL_GPIO_ReadPin(FEEDBACK_1_GPIO_Port, FEEDBACK_1_Pin);
	feedback_signals[1] = HAL_GPIO_ReadPin(FEEDBACK_2_GPIO_Port, FEEDBACK_2_Pin);
	feedback_signals[2] = HAL_GPIO_ReadPin(FEEDBACK_3_GPIO_Port, FEEDBACK_3_Pin);
	feedback_signals[3] = HAL_GPIO_ReadPin(FEEDBACK_4_GPIO_Port, FEEDBACK_4_Pin);
	feedback_signals[4] = HAL_GPIO_ReadPin(FEEDBACK_5_GPIO_Port, FEEDBACK_5_Pin);
	for (i = 0; i < 5; i++) {
		ret |= ((feedback_signals[i]&0x01)<<i);
	}
	return ret;
}

static uint8_t refresh_deploy_activation_status()
{
	uint8_t act_signals[8];
	uint8_t ret = 0;
	int i;
	act_signals[0] = HAL_GPIO_ReadPin(DEPLOY_1A_GPIO_Port, DEPLOY_1A_Pin);
	act_signals[1] = HAL_GPIO_ReadPin(DEPLOY_1B_GPIO_Port, DEPLOY_1B_Pin);
	act_signals[2] = HAL_GPIO_ReadPin(DEPLOY_2A_GPIO_Port, DEPLOY_2A_Pin);
	act_signals[3] = HAL_GPIO_ReadPin(DEPLOY_2B_GPIO_Port, DEPLOY_2B_Pin);
	act_signals[4] = HAL_GPIO_ReadPin(DEPLOY_3A_GPIO_Port, DEPLOY_3A_Pin);
	act_signals[5] = HAL_GPIO_ReadPin(DEPLOY_3B_GPIO_Port, DEPLOY_3B_Pin);
	act_signals[6] = HAL_GPIO_ReadPin(DEPLOY_4A_GPIO_Port, DEPLOY_4A_Pin);
	act_signals[7] = HAL_GPIO_ReadPin(DEPLOY_4B_GPIO_Port, DEPLOY_4B_Pin);
	for (i = 0; i < 8; i++) {
		ret |= ((act_signals[i]&0x01)<<i);
	}
	return ret;
}

uint8_t get_deployment_status()
{
	return (refresh_deploy_activation_status());
}

void command_answer_state()
{
	deploy_data_t data;
	taskENTER_CRITICAL();
	data.hk.deploy_status = get_feedbacks();
	data.hk.int_temp = get_internal_temperature();
	data.hk.ext_temp = get_external_temperature();
	data.control.deploys_activated = refresh_deploy_activation_status();
	data.control.deploys_error = get_deploy_error();
	taskEXIT_CRITICAL();
	i2c_slave_transmit(&data, sizeof(data));
}

void answer_null_command()
{
	deploy_data_t data;
	memset(&data, 0, sizeof(data));
	i2c_slave_transmit(&data, sizeof(data));
}

void process_incoming(deploy_command_t *c)
{
	uint8_t deploy_order;
	if (c->command == DEPLOY_GET_HK) {
		command_answer_state();
	}else if (c->command == DEPLOY_SET_DEPLOY) {
		deploy_order = c->deploy_activation;
		set_deploy_timer(c->deploy_timer);
		osSignalSet(controlTaskHandle, SIGNAL_OFFSET_NORMAL(deploy_order));
		command_answer_state();
	}else if (c->command == DEPLOY_UNSET_DEPLOY) {
		/* we have 4 deploys only... */
		osSignalSet(controlTaskHandle, SIGNAL_ABORT_DEPLOY);
		command_answer_state();
	}else if (c->command == DEPLOY_OVERRIDE_DEPLOY) {
		deploy_order = (c->deploy_activation);
		set_deploy_timer(c->deploy_timer);
		osSignalSet(controlTaskHandle, SIGNAL_OFFSET_OVERRIDE(deploy_order));
		command_answer_state();
	}else {
		answer_null_command();
	}
}


static void check_normal_deploy(uint32_t signals)
{
	uint16_t deploy_timeout;
	uint32_t tickstart;
	osEvent ret;
	if (signals & SIGNAL_DEPLOY_1A) {
	  set_deploy(DEPLOY_1, SIDE_A, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB1(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB1(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_1, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_1, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_1B) {
	  set_deploy(DEPLOY_1, SIDE_B, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB1(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB1(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_1, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_1, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_2A) {
	  set_deploy(DEPLOY_2, SIDE_A, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB2(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB2(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_2, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_2, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_2B) {
	  set_deploy(DEPLOY_2, SIDE_B, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB2(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB2(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_2, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_2, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_3A) {
	  set_deploy(DEPLOY_3, SIDE_A, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB3(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB3(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_3, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_3, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_3B) {
	  set_deploy(DEPLOY_3, SIDE_B, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB3(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB3(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_3, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_3, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_4A) {
	  set_deploy(DEPLOY_4, SIDE_A, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB4(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB4(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_4, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_4, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_4B) {
	  set_deploy(DEPLOY_4, SIDE_B, DEPLOY_NOT_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(  (UNDEPLOYED_FB4(get_feedbacks())) &&
			  ((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB4(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_4, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_4, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
}

static void check_override_deploy(uint32_t signals)
{
	osEvent ret;
	uint16_t deploy_timeout;
	uint32_t tickstart;
	if (signals & SIGNAL_DEPLOY_O_1A) {
	  set_deploy(DEPLOY_1, SIDE_A, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  if (UNDEPLOYED_FB1(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_1, SIDE_A, DEPLOY_ERR);
		  unset_deploy();
	  }else {
		  set_deploy_error(DEPLOY_1, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_1B) {
	  set_deploy(DEPLOY_1, SIDE_B, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB1(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_1, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_1, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_2A) {
	  set_deploy(DEPLOY_2, SIDE_A, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB2(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_2, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_2, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_2B) {
	  set_deploy(DEPLOY_2, SIDE_B, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB2(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_2, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_2, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_3A) {
	  set_deploy(DEPLOY_3, SIDE_A, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB3(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_3, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_3, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_3B) {
	  set_deploy(DEPLOY_3, SIDE_B, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB3(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_3, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_3, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_4A) {
	  set_deploy(DEPLOY_4, SIDE_A, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB4(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_4, SIDE_A, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_4, SIDE_A, DEPLOY_NO_ERR);
	  }
	}
	if (signals & SIGNAL_DEPLOY_O_4B) {
	  set_deploy(DEPLOY_4, SIDE_B, DEPLOY_OVERRIDE);
	  /* wait for that deploy to be performed */
	  tickstart = osKernelSysTick();
	  deploy_timeout = get_deploy_timer()*1000;
	  while(((osKernelSysTick() - tickstart) < deploy_timeout) ) {
		  /* 100 ms delay to test again... */
		  ret = osSignalWait(SIGNAL_ABORT_DEPLOY, 100);
		  if (ret.status == osEventSignal && (ret.value.signals & SIGNAL_ABORT_DEPLOY)) {
			unset_deploy(); return;
		  }
	  }
	  unset_deploy();
	  if (UNDEPLOYED_FB4(get_feedbacks())) {
		  /* if still undeployed, cancel it */
		  /* set ERROR on deploy variable */
		  set_deploy_error(DEPLOY_4, SIDE_B, DEPLOY_ERR);
	  }else {
		  set_deploy_error(DEPLOY_4, SIDE_B, DEPLOY_NO_ERR);
	  }
	}
}

void deploy_work(void)
{
	uint32_t signals;
	osEvent ret;
	signals = 0xFFFFFFFF;
	while(1) {
	  /* clear all signals */
	  ret = osSignalWait(signals, 1000);
	  if (ret.status == osEventSignal) {
		  /* if no deploy is enabled, enable 1 and only 1
		   * override if more than 1 enable is activated
		   * if more than 1 is tried to be activated, just avoid it...
		   */
		  if (get_deployment_status() == 0) {
			  check_normal_deploy(ret.value.signals);
			  check_override_deploy(ret.value.signals);
			  if (ret.value.signals & SIGNAL_ABORT_DEPLOY) {
				  unset_deploy();
			  }
		  }
	  }
	  refresh_temperatures();
	}
}

