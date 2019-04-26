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
			HAL_GPIO_WritePin(MOTOR_1_DIR_GPIOX,MOTOR_1_DIR_PINX,direction ? (GPIO_PIN_SET) : (GPIO_PIN_RESET));
			break;
	}
}

void SetEnableGPIO(STMotorHandle_t *STMotorHandle, bool flag){
	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
			HAL_GPIO_WritePin(MOTOR_1_ENABLE_GPIOX,MOTOR_1_ENABLE_PINX,flag ? (GPIO_PIN_RESET) : (GPIO_PIN_SET));
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

	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)"MOVE A OK",9);
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

	switch(GPIO_Pin){
	case MOTOR_1_ENDSTOP_PIN:
		STMotorEXTInterruptHandle(&STMotorDevices[0]);
		break;
	}
}

















