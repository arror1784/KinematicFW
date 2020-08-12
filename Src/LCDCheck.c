/*
 * LCDCheck.c
 *
 *  Created on: 2020. 7. 20.
 *      Author: HiX
 */

#include "LCDCheck.h"

volatile LCDState_t LCDState = LCD_NONE;

void setLCDState(LCDState_t ls){
	LCDState = ls;
}
LCDState_t getLCDState(){
	return LCDState;
}

