/*
 * HGCodeFunction.c
 *
 *  Created on: 2019. 2. 15.
 *      Author: JeonSeungHwan
 */
#include "HGCodeFunction.h"
#include "string.h"

extern STMotorHandle_t STMotorDevices[1];
extern HGCodeControl_t HGCodeControl;

HGCodeDataControl_t* temp = 0;
uint8_t commandCount = 0;
void startHGCode(TIM_HandleTypeDef* timHandler,UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle){

	HGCodeInit(HGCodeUsartHandle,HGCodeDmaHandle);
	HGCodeDMAStart();
	HAL_TIM_Base_Start_IT(timHandler);

	STMotorInitHandler(&STMotorDevices[0],&htim5,TIM_CHANNEL_1,MOTOR_1_ENDSTOP_IRQN);
	STMotorInitParam(&STMotorDevices[0],1000,400,MAX_SPEED,MIN_SPEED);

	while(1){
		if(HGCodeCheckDataBuffer() == 1){
			temp = HGCodeGetCommandData();
			if(temp->HGCodeCommand.G != 0){

				switch(temp->HGCodeCommand.G){
				case 01:
					G01();
					break;
				case 28:
					G28();
					break;
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
				case 100:
					H100();
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
			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 A 1 FAIL",12);
		}
		temp->HGCodeParameter.A = 0;
	}
	if(temp->HGCodeParameter.B){
		if(!STMotorIsActivate(&STMotorDevices[1])){
			STMotorGoMilli(&STMotorDevices[1],temp->HGCodeParameter.B);
			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 B 1 FAIL",12);
		}
		temp->HGCodeParameter.B = 0;
	}
	if(temp->HGCodeParameter.C){
		if(!STMotorIsActivate(&STMotorDevices[2])){
			STMotorGoMilli(&STMotorDevices[2],temp->HGCodeParameter.C);
			HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 C 1 FAIL",12);
		}
		temp->HGCodeParameter.C = 0;
	}
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G01 OK",6);
}
void G02(){
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
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G27 OK",6);
}
void G28(){
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"G28 OK",6);
	STMotorAutoHome(&STMotorDevices[0],0);
	//STMotorWaitingActivate(&STMotorDevices[0],0);

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
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H10 OK",6);
	HAL_GPIO_WritePin(UV_LCD_GPIO_Port,UV_LCD_Pin,GPIO_PIN_RESET);
}
void H11(){ //UV LED ON
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H11 OK",6);
	HAL_GPIO_WritePin(UV_LCD_GPIO_Port,UV_LCD_Pin,GPIO_PIN_SET);
}
void H20(){ //UV COOLER OFF
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H20 OK",6);
	HAL_GPIO_WritePin(COOLING_FAN_GPIO_Port,COOLING_FAN_Pin,GPIO_PIN_RESET);
}
void H21(){ //UV COOLER ON
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H21 OK",6);
	HAL_GPIO_WritePin(COOLING_FAN_GPIO_Port,COOLING_FAN_Pin,GPIO_PIN_SET);
}
void H100(){
	HAL_UART_Transmit_IT(HGCodeControl.HGCodeUartHandle,(uint8_t*)"H100 OK",7);
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
}

