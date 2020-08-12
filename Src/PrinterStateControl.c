/*
 * PrinterStateControl.c
 *
 *  Created on: 2020. 5. 23.
 *      Author: hix
 */

#include "PrinterStateControl.h"

#include "main.h"
#include "HGCodeFunction.h"
#include "StepMotorDriver.h"
#include "common.h"

PowerState_t PState = POWER_OFF;

uint8_t lk = 1;

void setPowerState(PowerState_t st){
	PState = st;
}

PowerState_t getPowerState(){
	return PState;
}

void powerOn(){
	setPowerState(POWER_ON);

	STMotorDeviceControl.SetEnableGPIO(&STMotorDevices[0],TRUE);
	HAL_GPIO_WritePin(BOOT_GPIO_Port,BOOT_Pin,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
}
void powerOff(){
	setPowerState(POWER_OFF);

	STMotorDeviceControl.SetEnableGPIO(&STMotorDevices[0],FALSE);
	HAL_GPIO_WritePin(BOOT_GPIO_Port,BOOT_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim10,TIM_CHANNEL_1);
}
void softPowerOff(){
	lk = 0;

	HAL_UART_Transmit(&huart3,(uint8_t*)"POWER OFF after 17 sec\r\n",11,1000);
	HAL_Delay(17000);

	STMotorDeviceControl.SetEnableGPIO(&STMotorDevices[0],FALSE);
	HAL_GPIO_WritePin(BOOT_GPIO_Port,BOOT_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim10,TIM_CHANNEL_1);
	setPowerState(POWER_OFF);

	lk = 1;
}
void controlPowerStateBTN(BtnChannel_t btn){
	PowerState_t st = getPowerState();
	if(lk){
		switch(btn){
			case BTN_FRT_SHORT:
				if(st == POWER_ON){
				break;
			}else if(st == POWER_OFF){
				HAL_UART_Transmit(&huart3,(uint8_t*)"POWER ON\r\n",10,1000);
				powerOn();
				break;
			}
			break;
			case BTN_FRT_LONG:
				if(st == POWER_ON){
					//send signal power off
					sendResponse(0,200,100);
					HAL_UART_Transmit(&huart3,(uint8_t*)"SEND SIGNAL\r\n",13,1000);
					break;
				}else if(st == POWER_OFF){
					HAL_UART_Transmit(&huart3,(uint8_t*)"POWER ON\r\n",10,1000);
					powerOn();
					break;
				}
		}
	}
	if(btn == BTN_FRT_LONG_LONG){
		HAL_UART_Transmit(&huart3,(uint8_t*)"LONG LONG POWER OFF\r\n",21,1000);
		powerOff();
	}
}
