/*
 * HGCodeFunction.c
 *
 *  Created on: 2019. 2. 15.
 *      Author: JeonSeungHwan
 */
#include <LAPSRCcommon.h>
#include "HGCodeFunction.h"
#include "StepMotorDriver.h"
#include "PrinterStateControl.h"
#include "string.h"
#include "tim.h"
#include "neoPixel.h"
#include "usart.h"

#include "menu.h"
#include "LAPSRCcommon.h"

extern HGCodeControl_t HGCodeControl;

void startHGCode(TIM_HandleTypeDef* timHandler,UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle){

	HGCodeInit(HGCodeUsartHandle,HGCodeDmaHandle);
	HGCodeDMAStart();
	HAL_TIM_Base_Start_IT(timHandler);

}
void G01(HGCodeDataControl_t* temp){
	if(temp->HGCodeParameter.A){
		if(!STMotorIsActivate(&STMotorDevices[0])){
			STMotorGoMicro(&STMotorDevices[0],temp->HGCodeParameter.A,temp->HGCodeParameter.M);
		}else{
//			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 A 1 FAIL",12);
		}
		temp->HGCodeParameter.A = 0;
	}
	//HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 OK",6);
}
void G02(HGCodeDataControl_t* temp){
	if(temp->HGCodeParameter.A){
			if(!STMotorIsActivate(&STMotorDevices[0])){
				STMotorGoStep(&STMotorDevices[0],STMotorCalcMicroToStep(temp->HGCodeParameter.A) - STMotorDevices[0].motorParam.curPosition,temp->HGCodeParameter.M);
//				STMotorGoMilli(&STMotorDevices[0],temp->HGCodeParameter.A);
			}else{
//				HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 A 1 FAIL",12);
		}
		temp->HGCodeParameter.A = 0;
	}
	return;
}
void G03(HGCodeDataControl_t* temp){
	if(temp->HGCodeParameter.A){
			if(!STMotorIsActivate(&STMotorDevices[0])){
				STMotorGoStep(&STMotorDevices[0],temp->HGCodeParameter.A,temp->HGCodeParameter.M);
			}else{
//				HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 A 1 FAIL",12);
		}
		temp->HGCodeParameter.A = 0;
	}
	return;
}
void G04(HGCodeDataControl_t* temp){
	return;
}
void G05(HGCodeDataControl_t* temp){
	return;
}
void G27(HGCodeDataControl_t* temp){
	STMotorGoHome(&STMotorDevices[0]);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G27 OK",6);
}
void G28(HGCodeDataControl_t* temp){
//	STMotorGoMilli(&STMotorDevices[0],5);
	if(!STMotorIsActivate(&STMotorDevices[0])){
		if(temp->HGCodeParameter.A == 0)
			STMotorAutoHome(&STMotorDevices[0],0);
		else
			STMotorAutoHome(&STMotorDevices[0],(int32_t)((temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH_MIL / ((double)MOTOR_MICRO_STEP * (double)MOTOR_MOTOR_STEP))));
	}else
		HAL_UART_Transmit(&huart3,(uint8_t*)"is activate\r\n",14,1000);
}
void G30(HGCodeDataControl_t* temp){
	return;
}
void G31(HGCodeDataControl_t* temp){
	return;
}
void G99(HGCodeDataControl_t* temp){
	return;
}


void H01(HGCodeDataControl_t* temp){
	return;
}
void H05(HGCodeDataControl_t* temp){
	return;
}
void H10(HGCodeDataControl_t* temp){ //UV LED OFF
//	HAL_GPIO_WritePin(UV_LED_MIDDLE_GPIO_Port,UV_LED_MIDDLE_Pin,GPIO_PIN_RESET);
	  HAL_TIM_PWM_Stop_IT(&htim12,TIM_CHANNEL_2);
}
void H11(HGCodeDataControl_t* temp){ //UV LED ON
//	HAL_GPIO_WritePin(UV_LED_MIDDLE_GPIO_Port,UV_LED_MIDDLE_Pin,GPIO_PIN_SET);
	  HAL_TIM_PWM_Start_IT(&htim12,TIM_CHANNEL_2);
}
void H12(HGCodeDataControl_t* temp){ //SET UV LED PWM
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, temp->HGCodeParameter.A);
}

void H20(HGCodeDataControl_t* temp){ //UV COOLER OFF
	HAL_TIM_Base_Stop_IT(&htim10);
}
void H21(HGCodeDataControl_t* temp){ //UV COOLER ON
	HAL_TIM_Base_Start_IT(&htim10);
}
void H22(HGCodeDataControl_t* temp){ //SET UV COOLER PWM
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, temp->HGCodeParameter.A);
}
//H3x : 입력값 : 분당 밀리미터 단위
void H30(HGCodeDataControl_t* temp){ //SET MAX SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetMaxSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetMaxSpeed(&STMotorDevices[0],(uint32_t)(((double)temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH_MIL / ((double)MOTOR_MICRO_STEP * (double)MOTOR_MOTOR_STEP))),temp->HGCodeParameter.M);
		temp->HGCodeParameter.A = 0;
	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H30 OK",6);
}
void H31(HGCodeDataControl_t* temp){ //SET MIN SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetMinSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetMinSpeed(&STMotorDevices[0],(uint32_t)(((double)temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH_MIL / ((double)MOTOR_MICRO_STEP * (double)MOTOR_MOTOR_STEP))),temp->HGCodeParameter.M);

		temp->HGCodeParameter.A = 0;
	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H31 OK",6);
}
void H32(HGCodeDataControl_t* temp){ //SET ACCEL SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetAccelSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetAccelSpeed(&STMotorDevices[0],(uint32_t)(((double)temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH_MIL / ((double)MOTOR_MICRO_STEP * (double)MOTOR_MOTOR_STEP))),temp->HGCodeParameter.M);

		temp->HGCodeParameter.A = 0;
	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H32 OK",6);
}
void H33(HGCodeDataControl_t* temp){ //SET DECEL SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetDecelSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetDecelSpeed(&STMotorDevices[0],(uint32_t)(((double)temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH_MIL / ((double)MOTOR_MICRO_STEP * (double)MOTOR_MOTOR_STEP))),temp->HGCodeParameter.M);
		temp->HGCodeParameter.A = 0;
	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H33 OK",6);
}
void H40(HGCodeDataControl_t* temp){
//	uint8_t buff[50] = {0};
//	if(temp->HGCodeParameter.A){
//		sprintf(buff,"max speed %d\r\n",STMotorDevices[0].motorParam.maxSpeed);
//		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//		HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20);
//	}
}
void H41(HGCodeDataControl_t* temp){
//	uint8_t buff[50] = {0};
//	if(temp->HGCodeParameter.A){
//		sprintf(buff,"min speed %d\r\n",STMotorDevices[0].motorParam.minSpeed);
//		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//	}
}
void H42(HGCodeDataControl_t* temp){
//	uint8_t buff[50] = {0};
//	if(temp->HGCodeParameter.A){
//		sprintf(buff,"accel speed %d\r\n",STMotorDevices[0].motorParam.accel);
//		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//	}
}
void H43(HGCodeDataControl_t* temp){
//	uint8_t buff[50] = {0};
//	if(temp->HGCodeParameter.A){
//		sprintf(buff,"decel speed %lu\r\n",	STMotorDevices[0].motorParam.decel);
//		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//	}
}
void H44(HGCodeDataControl_t* temp){
//	uint8_t buff[50] = {0};
//	if(temp->HGCodeParameter.A){
//		sprintf(buff,"current position %lu\r\n",	STMotorDevices[0].motorParam.curPosition);
//		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//	}
}

void H50(HGCodeDataControl_t* temp,WS2812BControl_p neoPixel_P){

	for(int i = 0 ; i < neoPixel_P->ledCount; i++){
		setColor(neoPixel_P,converColorTo32((uint8_t)temp->HGCodeParameter.B,(uint8_t)temp->HGCodeParameter.A,(uint8_t)temp->HGCodeParameter.C),i);
	}
	updateColor(neoPixel_P,0);
}
void H51(HGCodeDataControl_t* temp,WS2812BControl_p neoPixel_P){
//	uint32_t buff[10 + 2 * 24] = { 0 };
	for(int i = 0 ; i < neoPixel_P->ledCount; i++){
//		setColorArr(&buff[24],converColorTo32((uint8_t)temp->HGCodeParameter.B,(uint8_t)temp->HGCodeParameter.A,(uint8_t)temp->HGCodeParameter.C),i);
		setColor(neoPixel_P,converColorTo32((uint8_t)temp->HGCodeParameter.B,(uint8_t)temp->HGCodeParameter.A,(uint8_t)temp->HGCodeParameter.C),i);
	}
	updateColor(neoPixel_P,1);
}
void H60(HGCodeDataControl_t* temp){
//	uint8_t buff[50] = {0};
//	sprintf(buff,"current position %ld \r\n",STMotorDevices[0].motorParam.curPosition);
//	HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
}
void H90(HGCodeDataControl_t* temp){
	return;
}
void H91(HGCodeDataControl_t* temp){
	if(HAL_GPIO_ReadPin(LCD_CHECK_GPIO_Port,LCD_CHECK_Pin) == GPIO_PIN_SET){
		sendResponse(0,91,0);
	}else{
		sendResponse(0,91,1);
	}

	return;
}
void H101(HGCodeDataControl_t* temp){

	int8_t k=0;
	uint8_t buf[10] = {0};
	HGCodeDMAPause();
//	while (1){
//		if(SerialKeyPressed(&k)){
	k = SerialDownload_backup();
	if(k == 0){
		sprintf(buf,"%d\r\n",k);
		HAL_UART_Transmit(&huart3,"YMODEM SUCESS\r\n",15,1000);
		HAL_UART_Transmit(&huart3,buf,10,1000);
//		break;
	}else{
		sprintf(buf,"%d\r\n",k);
		HAL_UART_Transmit(&huart3,"YMODEM ERROR\r\n",14,1000);
		HAL_UART_Transmit(&huart3,buf,10,1000);
//		break;
	}
//		}
//	}
	HGCodeDMAResume();
	HAL_UART_Transmit(&huart3,"YMODEM FINISH\r\n",15,1000);
//	pFunction Jump_To_Application;
//	uint32_t JumpAddress;
//	HAL_UART_Transmit(&huart3,"IAPr\n",5,1000);
//
//	JumpAddress = *(__IO uint32_t*) ((uint32_t)0x08000000 + 4);
//	/* Jump to user application */
//	Jump_To_Application = (pFunction) JumpAddress;
//	/* Initialize user application's Stack Pointer */
//	__set_MSP(*(__IO uint32_t*) (uint32_t)0x08000000);
//	Jump_To_Application();
}
void H200(HGCodeDataControl_t* temp){
	softPowerOff();
}
void H201(HGCodeDataControl_t* temp){
	softPowerOff();
	NVIC_SystemReset();
}
void sendResponse(uint8_t bed,uint8_t command,uint8_t response){

	uint8_t buff[10] = {0};

	buff[0] = 0x02;
	buff[1] = bed;
	buff[2] = command;
	buff[3] = response;
	buff[4] = (uint8_t)(buff[1] + buff[2] + buff[3]);
	buff[5] = 0x03;

	while(HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,buff,6,1000) != HAL_OK);

}
