/*
 * PowerBTN.c
 *
 *  Created on: 2020. 11. 5.
 *      Author: hix
 */
#include "PowerBTN.h"
#include "stm32f4xx_hal.h"
#include "PrinterStateControl.h"
#include "main.h"

void BTNEXTIHandle(){
	static uint32_t count = 0;

	if(count == 0){
		if(HAL_GPIO_ReadPin(FRT_BUTTON_GPIO_Port,FRT_BUTTON_Pin) == GPIO_PIN_RESET){
			count = 1;
		}
	}else if(count == DEBOUNCING_SHORT_TIME){
		if(HAL_GPIO_ReadPin(FRT_BUTTON_GPIO_Port,FRT_BUTTON_Pin) == GPIO_PIN_RESET){
			count += 1;
		}else{
			count = 0;
		}
	}else{
		count += 1;

		if(count == DEBOUNCING_SHORT_TIME /*&& count < DEBOUNCING_LONG_TIME*/){ //short
			controlPowerStateBTN(BTN_FRT_SHORT);
		}else if(count ==DEBOUNCING_LONG_TIME){ // long
			controlPowerStateBTN(BTN_FRT_LONG);
		}else if(count == DEBOUNCING_LONG_LONG_TIME){
			controlPowerStateBTN(BTN_FRT_LONG_LONG);
		}
		if(HAL_GPIO_ReadPin(FRT_BUTTON_GPIO_Port,FRT_BUTTON_Pin) != GPIO_PIN_RESET){
			count = 0;
		}
	}
}
