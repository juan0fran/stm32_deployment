/*
 * deploy_execution.h
 *
 *  Created on: 2 de oct. de 2017
 *      Author: cubecat
 */

#ifndef DEPLOY_EXECUTION_H_
#define DEPLOY_EXECUTION_H_

#include "main.h"
#include "stm32l0xx_hal.h"

#define DEPLOYED	1
#define UNDEPLOYED	0

#define DEPLOY_ERR	 	1
#define DEPLOY_NO_ERR 	0

#define DEPLOY_NOT_OVERRIDE		0
#define DEPLOY_OVERRIDE 		1

#define DEPLOYED_FB1(x)			(((x>>0)&0x01))
#define DEPLOYED_FB2(x)			(((x>>1)&0x01))
#define DEPLOYED_FB3(x)			(((x>>2)&0x01))
#define DEPLOYED_FB4(x)			(((x>>3)&0x01))

#define UNDEPLOYED_FB1(x)		(!((x>>0)&0x01))
#define UNDEPLOYED_FB2(x)		(!((x>>1)&0x01))
#define UNDEPLOYED_FB3(x)		(!((x>>2)&0x01))
#define UNDEPLOYED_FB4(x)		(!((x>>3)&0x01))

typedef struct __attribute__ ((__packed__)) deploy_data_u { /* 6 bytes */
	struct __attribute__ ((__packed__)) {
		uint8_t 	deploy_status;
		uint16_t 	int_temp;
		uint16_t 	ext_temp;
	}hk;
	struct __attribute__ ((__packed__)) {
		/* 8 bits for 8 deploys, 4 deploys+sideA/B */
		uint8_t		deploys_activated;
		uint8_t		deploys_error;
	}control;
}deploy_data_t;

typedef struct __attribute__ ((__packed__)) deploy_command_u { /* 2 bytes */
	uint8_t 	command;
	uint8_t 	deploy_activation;
	uint8_t		deploy_timer;
}deploy_command_t;

typedef enum deploy_num_e {
	DEPLOY_1 = 0x1,
	DEPLOY_2 = 0x2,
	DEPLOY_3 = 0x4,
	DEPLOY_4 = 0x8,
}deploy_num_e;

typedef enum deploy_side_e {
	SIDE_A,
	SIDE_B,
}deploy_side_e;

typedef enum deploy_command_e {
	DEPLOY_GET_HK,
	DEPLOY_SET_DEPLOY,
	DEPLOY_UNSET_DEPLOY,
	DEPLOY_OVERRIDE_DEPLOY,
}deploy_command_e;

void 	set_deploy(deploy_num_e which, deploy_side_e side, int deploy_override);
void 	unset_deploy();

uint8_t get_feedbacks();
uint8_t get_deployment_status();

uint8_t get_deploy_timer(void);
void 	set_deploy_timer(uint8_t timer);

void 	process_incoming(deploy_command_t *c);

void 	set_deploy_error(deploy_num_e which, deploy_side_e side, uint8_t set_unset);

void 	deploy_work(void);

#endif /* DEPLOY_EXECUTION_H_ */

