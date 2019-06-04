/*
 * A4988.c
 *
 *  Created on: 2019. 1. 28.
 *      Author: JeonSeungHwan
 */
#include "StepMotorDriver.h"

#include "common.h"
#include "PWMStepMotor.h"
#include "usart.h"


STMotorHandle_t STMotorDevices[4];

STMotorDeviceControl_t STMotorDeviceControl = {
	SetDirectionGPIO,
	SetEnableGPIO,
	NULL,
	NULL,
	NULL,
	NULL,
	FinishCallBack
};

void SetDirectionGPIO(STMotorHandle_t *STMotorHandle, MotorDirection_t direction){
	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
			HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port,MOTOR_DIR_1_Pin,direction ? (GPIO_PIN_SET) : (GPIO_PIN_RESET));
			break;
	}
}

void SetEnableGPIO(STMotorHandle_t *STMotorHandle, bool flag){
	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
			HAL_GPIO_WritePin(MOTOR_EN_1_GPIO_Port,MOTOR_EN_1_Pin,flag ? (GPIO_PIN_RESET) : (GPIO_PIN_SET));
			break;
	}
}
void SetSleepGPIO(STMotorHandle_t *STMotorHandle, bool flag){
	return;
}
void SetResetGPIO(STMotorHandle_t *STMotorHandle,bool flag){
	return;
}

void SetMicroStep(STMotorHandle_t *STMotorHandle,uint16_t microStep){
	return;
}

void SetEnableIRQ(STMotorHandle_t *STMotorHandle,bool flag){
	return;
}


void FinishCallBack(STMotorHandle_t *STMotorHandle){
	uint8_t response[15];
	uint8_t buff[50] = {0};
	response[0] = 0x02;
	response[1] = 'M';
	response[2] = 'O';
	response[3] = 'V';
	response[4] = 'E';
	response[5] = ' ';
	response[6] = 'A';
	response[7] = ' ';
	response[8] = 'O';
	response[9] = 'K';
	response[10] = 0x52;
	response[11] = 0x03;

	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
			HAL_UART_Transmit_IT(&huart2,response,12);
			sprintf(buff,"current position %ld\r\n",STMotorHandle->motorParam.curPosition);
			HAL_UART_Transmit(&huart1,(uint8_t*)buff,40,1000);
			break;
		case 0x01:
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)"MOVE B OK",9);
			break;
		case 0x02:
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)"MOVE C OK",9);
			break;
		case 0x03:
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)"MOVE P OK",9);
			break;
		default:
			break;
	}

	return;
}


void PWMPulseInterruptHandle(TIM_HandleTypeDef *htim){
	if(htim->Instance == STMotorDevices[0].motorHandler.instance){
		STMotorPWMPulseInterruptHandle(&STMotorDevices[0]);
	}
}

void EXTInterruptHandle(uint16_t GPIO_Pin){

	uint8_t response[9];
	response[0] = 0x02;
	response[1] = 's';
	response[2] = 't';
	response[3] = 'a';
	response[4] = 'r';
	response[5] = 't';
	response[6] = 0x2E;
	response[7] = 0x03;

	switch(GPIO_Pin){
	case MOTOR_1_ENDSTOP_PIN:
		STMotorEXTInterruptHandle(&STMotorDevices[0]);
		HAL_UART_Transmit_IT(&huart1,"end stop interrupt\r\n",40);
		break;
//	case POWER_BTN_Pin:
//		HAL_UART_Transmit(&huart2,response,8,1000);
//		HAL_UART_Transmit(&huart1,"FRNT BTN interrupt\r\n",40,1000);
//		break;
	}
}

