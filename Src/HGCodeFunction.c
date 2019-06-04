/*
 * HGCodeFunction.c
 *
 *  Created on: 2019. 2. 15.
 *      Author: JeonSeungHwan
 */
#include "HGCodeFunction.h"
#include "string.h"
#include "tim.h"
#include "neoPixel.h"
#include "usart.h"

extern STMotorHandle_t STMotorDevices[1];
extern HGCodeControl_t HGCodeControl;
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;

HGCodeDataControl_t* temp = 0;
uint8_t commandCount = 0;
WS2812BControl_t neoPixel = {0};
P_WS2812BControl_t neoPixel_P = &neoPixel;

void startHGCode(TIM_HandleTypeDef* timHandler,UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle){

	uint8_t buff[100] = {0};

	HGCodeInit(HGCodeUsartHandle,HGCodeDmaHandle);
	HGCodeDMAStart();
	HAL_TIM_Base_Start_IT(timHandler);

	STMotorInitHandler(&STMotorDevices[0],&htim9,TIM_CHANNEL_1,EXTI9_5_IRQn);
	STMotorInitParam(&STMotorDevices[0],400,400,MAX_SPEED,MIN_SPEED);

	setNeoPixel(neoPixel_P,&htim1,TIM_CHANNEL_4,&hdma_tim1_ch4_trig_com,10);

	while(1){
		if(HGCodeCheckDataBuffer() == 1){
			temp = HGCodeGetCommandData();

			sprintf(buff,"command count: %4d, G: %4d, H: %4d, A: %lf\r\n",
					commandCount++,temp->HGCodeCommand.G,temp->HGCodeCommand.H,temp->HGCodeParameter.A);
			HAL_UART_Transmit_IT(&huart1,buff,strlen(buff));

			if(temp->HGCodeCommand.G != 0){
				switch(temp->HGCodeCommand.G){
				case 01:
					G01();
					break;
				case 02:
					G02();
					break;
				case 28:
					G28();
					break;
				case 27:
					G27();
				default:
					break;
				}
//				temp->HGCodeCommand.G = 0;
			}else if(temp->HGCodeCommand.H != 0){
				switch(temp->HGCodeCommand.H){
				case 10:
					H10();
					break;
				case 11:
					H11();
					break;
				case 20:
					H20();
					break;
				case 21:
					H21();
					break;
				case 30:
					H30();
					break;
				case 31:
					H31();
					break;
				case 32:
					H32();
					break;
				case 33:
					H33();
					break;
				case 40:
					H40();
					break;
				case 41:
					H41();
					break;
				case 42:
					H42();
					break;
				case 43:
					H43();
					break;
				case 50:
					H50();
					break;
				case 100:
					H100();
					break;
				case 60:
					H60();
					break;
				default:
					break;
				}
//				temp->HGCodeCommand.H = 0;
			}
			memset(temp,0x00,sizeof(HGCodeDataControl_t));
		}
	}
}
void G01(){
	if(temp->HGCodeParameter.A){
		if(!STMotorIsActivate(&STMotorDevices[0])){
			STMotorGoMilli(&STMotorDevices[0],temp->HGCodeParameter.A);
		}else{
//			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 A 1 FAIL",12);
		}
		temp->HGCodeParameter.A = 0;
	}
	if(temp->HGCodeParameter.B){
		if(!STMotorIsActivate(&STMotorDevices[1])){
			STMotorGoMilli(&STMotorDevices[1],temp->HGCodeParameter.B);
		}else{
//			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 B 1 FAIL",12);
		}
		temp->HGCodeParameter.B = 0;
	}
	if(temp->HGCodeParameter.C){
		if(!STMotorIsActivate(&STMotorDevices[2])){
			STMotorGoMilli(&STMotorDevices[2],temp->HGCodeParameter.C);
		}else{
//			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 C 1 FAIL",12);
		}
		temp->HGCodeParameter.C = 0;
	}
	//HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 OK",6);
}
void G02(){
	if(temp->HGCodeParameter.A){
			if(!STMotorIsActivate(&STMotorDevices[0])){
				STMotorGoStep(&STMotorDevices[0],STMotorCalcMilliToStep(temp->HGCodeParameter.A) - STMotorDevices[0].motorParam.curPosition);
//				STMotorGoMilli(&STMotorDevices[0],temp->HGCodeParameter.A);
			}else{
//				HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 A 1 FAIL",12);
		}
		temp->HGCodeParameter.A = 0;
	}
	return;
}
void G04(){
	return;
}
void G05(){
	return;
}
void G27(){
	STMotorGoHome(&STMotorDevices[0]);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G27 OK",6);
}
void G28(){
//	STMotorGoMilli(&STMotorDevices[0],5);
	if(!STMotorIsActivate(&STMotorDevices[0])){
		if(temp->HGCodeParameter.A == 0)
			STMotorAutoHome(&STMotorDevices[0],0);
		else
			STMotorAutoHome(&STMotorDevices[0],(int32_t)((temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH / ((double)MOTOR_DRIVER_MiCRO_STEP * (double)MOTOR_STEP))));
	}else
		while(HAL_UART_Transmit_IT(&huart1,"is activate\r\n",13) != HAL_OK);
//		HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"MOVE A OK",9);
//		HAL_UART_Transmit_IT(&huart1,"is activate\r\n",13);
}
void G30(){
	return;
}
void G31(){
	return;
}
void G99(){
	return;
}
void H01(){
	return;
}
void H05(){
	return;
}
void H10(){ //UV LED OFF
	HAL_GPIO_WritePin(UV_LED_GPIO_Port,UV_LED_Pin,GPIO_PIN_RESET);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H10 OK",6);
}
void H11(){ //UV LED ON
	HAL_GPIO_WritePin(UV_LED_GPIO_Port,UV_LED_Pin,GPIO_PIN_SET);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H11 OK",6);
}
void H20(){ //UV COOLER OFF
	HAL_GPIO_WritePin(COOLING_FAN_GPIO_Port,COOLING_FAN_Pin,GPIO_PIN_RESET);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H20 OK",6);
}
void H21(){ //UV COOLER ON
	HAL_GPIO_WritePin(COOLING_FAN_GPIO_Port,COOLING_FAN_Pin,GPIO_PIN_SET);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H21 OK",6);
}
void H30(){ //SET MAX SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetMaxSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetMaxSpeed(&STMotorDevices[0],(uint32_t)((temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH / ((double)MOTOR_DRIVER_MiCRO_STEP * (double)MOTOR_STEP))));
		temp->HGCodeParameter.A = 0;
	}
	if(temp->HGCodeParameter.B){
		STMotorSetMaxSpeed(&STMotorDevices[1],temp->HGCodeParameter.B);
		temp->HGCodeParameter.B = 0;
	}
	if(temp->HGCodeParameter.C){
		STMotorSetMaxSpeed(&STMotorDevices[2],temp->HGCodeParameter.B);
		temp->HGCodeParameter.C = 0;
	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H30 OK",6);
}
void H31(){ //SET MIN SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetMinSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetMinSpeed(&STMotorDevices[0],(uint32_t)((temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH / ((double)MOTOR_DRIVER_MiCRO_STEP * (double)MOTOR_STEP))));

		temp->HGCodeParameter.A = 0;
	}
//	if(temp->HGCodeParameter.B){
//		STMotorSetMinSpeed(&STMotorDevices[1],temp->HGCodeParameter.B);
//		temp->HGCodeParameter.B = 0;
//	}
//	if(temp->HGCodeParameter.C){
//		STMotorSetMinSpeed(&STMotorDevices[2],temp->HGCodeParameter.B);
//		temp->HGCodeParameter.C = 0;
//	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H31 OK",6);
}
void H32(){ //SET ACCEL SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetAccelSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetAccelSpeed(&STMotorDevices[0],(uint32_t)((temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH / ((double)MOTOR_DRIVER_MiCRO_STEP * (double)MOTOR_STEP))));

		temp->HGCodeParameter.A = 0;
	}
//	if(temp->HGCodeParameter.B){
//		STMotorSetAccelSpeed(&STMotorDevices[1],temp->HGCodeParameter.B);
//		temp->HGCodeParameter.B = 0;
//	}
//	if(temp->HGCodeParameter.C){
//		STMotorSetAccelSpeed(&STMotorDevices[2],temp->HGCodeParameter.B);
//		temp->HGCodeParameter.C = 0;
//	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H32 OK",6);
}
void H33(){ //SET DECEL SPEED
	if(temp->HGCodeParameter.A){
//		STMotorSetDecelSpeed(&STMotorDevices[0],temp->HGCodeParameter.A);
		STMotorSetDecelSpeed(&STMotorDevices[0],(uint32_t)((temp->HGCodeParameter.A / 60 ) / ((double)MOTOR_SCREW_PITCH / ((double)MOTOR_DRIVER_MiCRO_STEP * (double)MOTOR_STEP))));

		temp->HGCodeParameter.A = 0;
	}
//	if(temp->HGCodeParameter.B){
//		STMotorSetDecelSpeed(&STMotorDevices[1],temp->HGCodeParameter.B);
//		temp->HGCodeParameter.B = 0;
//	}
//	if(temp->HGCodeParameter.C){
//		STMotorSetDecelSpeed(&STMotorDevices[2],temp->HGCodeParameter.B);
//		temp->HGCodeParameter.C = 0;
//	}
	HAL_Delay(5);
//	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H33 OK",6);
}
void H40(){
	uint8_t buff[50] = {0};
	if(temp->HGCodeParameter.A){
		sprintf(buff,"max speed %d\r\n",STMotorDevices[0].motorParam.maxSpeed);
		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//		HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20);

	}
}
void H41(){
	uint8_t buff[50] = {0};
	if(temp->HGCodeParameter.A){
		sprintf(buff,"min speed %d\r\n",STMotorDevices[0].motorParam.minSpeed);
		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
	}
}
void H42(){
	uint8_t buff[50] = {0};
	if(temp->HGCodeParameter.A){
		sprintf(buff,"accel speed %d\r\n",STMotorDevices[0].motorParam.accel);
		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
	}
}
void H43(){
	uint8_t buff[50] = {0};
	if(temp->HGCodeParameter.A){
		sprintf(buff,"decel speed %lu\r\n",	STMotorDevices[0].motorParam.decel);
		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
	}
}
//void H44(){
//	uint8_t buff[50] = {0};
//	if(temp->HGCodeParameter.A){
//		sprintf(buff,"current position %lu\r\n",	STMotorDevices[0].motorParam.curPosition);
//		HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
//	}
//}

void H50(){
	for(int i = 0 ; i < neoPixel_P->ledCount; i++){
		setColor(neoPixel_P,converColorTo32((uint8_t)temp->HGCodeParameter.B,(uint8_t)temp->HGCodeParameter.A,(uint8_t)temp->HGCodeParameter.C),i);
	}
	updateColor(neoPixel_P);
}
void H60(){
	uint8_t buff[50] = {0};
	sprintf(buff,"current position %ld \r\n",STMotorDevices[0].motorParam.curPosition);
	HAL_UART_Transmit(HGCodeControl.HGCodeUartHandle,(uint8_t*)buff,20,1000);
}
void H100(){
	uint8_t buff[6];
	buff[0] = 0x02;
	buff[1] = 'O';
	buff[2] = 'K';
	buff[3] = 0x9A;
	buff[4] = 0x03;

	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	while(HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,buff,5) != HAL_OK);
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)"H100 OK\r\n",9);
}

