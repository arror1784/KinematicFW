/*
 * A4988.c
 *
 *  Created on: 2019. 1. 28.
 *      Author: JeonSeungHwan
 */
#include "StepMotorDriver.h"

#include "common.h"
#include "PWMStepMotor.h"
#include "HGCodeFunction.h"
#include "usart.h"
#include <stdio.h>


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
			HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port,MOTOR_DIR_Pin,direction ? (GPIO_PIN_SET) : (GPIO_PIN_RESET));
			break;
	}
}

void SetEnableGPIO(STMotorHandle_t *STMotorHandle, bool flag){
	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
			HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,MOTOR_EN_Pin,flag ? (GPIO_PIN_RESET) : (GPIO_PIN_SET));
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
	uint8_t buff[50] = {0};

	switch(STMotorHandle->motorHandler.deviceNumber){
		case 0x00:
//			HAL_UART_Transmit_IT(&huart2,response,12);
			sendResponse(1,101,100);
			sprintf((char*)buff,(char*)"current position %ld\r\n",STMotorHandle->motorParam.curPosition);
			while(HAL_UART_Transmit(&huart3,(uint8_t*)buff,40,1000) != HAL_OK);
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

void EXTInterruptHandle(BtnChannel_t btn){

	switch(btn){
	case BTN_END:
		STMotorEXTInterruptHandle(&STMotorDevices[0]);
		HAL_UART_Transmit(&huart3,(uint8_t*)"end stop\r\n",10,1000);
		break;
	case BTN_FRT_SHORT:
//		HAL_UART_Transmit(&huart2,response,8,1000);
		sendResponse(0,102,100);
		HAL_UART_Transmit(&huart3,(uint8_t*)"BTN SHORT\r\n",11,1000);
		break;
	case BTN_FRT_LONG:
		sendResponse(0,103,100);
		HAL_UART_Transmit(&huart3,(uint8_t*)"BTN LONG\r\n",10,1000);
	}

}

