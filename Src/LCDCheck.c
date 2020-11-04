/*
 * LCDCheck.c
 *
 *  Created on: 2020. 7. 20.
 *      Author: HiX
 */

#include "LCDCheck.h"
#include "main.h"
#include "usart.h"
#include "HGCodeFunction.h"

volatile LCDState_t LCDState = LCD_NONE;

void setLCDState(LCDState_t ls){
	LCDState = ls;
}
LCDState_t getLCDState(){
	return LCDState;
}

void LCDCheckEXTIHandle(){
	static uint8_t LCDFirstCheck = 0;

	if(LCDFirstCheck == 0){
		LCDFirstCheck = 1;
		if(HAL_GPIO_ReadPin(LCD_CHECK_GPIO_Port,LCD_CHECK_Pin) == GPIO_PIN_SET){
			setLCDState(LCD_OFF);
		}else if(HAL_GPIO_ReadPin(LCD_CHECK_GPIO_Port,LCD_CHECK_Pin) == GPIO_PIN_RESET){
			setLCDState(LCD_ON);
		}
	}else{
		if(HAL_GPIO_ReadPin(LCD_CHECK_GPIO_Port,LCD_CHECK_Pin) == GPIO_PIN_SET){
			if(getLCDState() == LCD_ON){
				//lcd state change to LCD_OFF
				sendResponse(0,91,0);
				HAL_UART_Transmit(&huart3,(uint8_t*)"LCD OFF\r\n",9,1000);
				setLCDState(LCD_OFF);
			}
		}else/* if(HAL_GPIO_ReadPin(LCD_CHECK_GPIO_Port,LCD_CHECK_Pin) == GPIO_PIN_RESET)*/
		{
			if(getLCDState() == LCD_OFF){
				//lcd state change to LCD_ON
				sendResponse(0,91,1);
				HAL_UART_Transmit(&huart3,(uint8_t*)"LCD ON\r\n",8,1000);
				setLCDState(LCD_ON);
			}
		}
	}
}
