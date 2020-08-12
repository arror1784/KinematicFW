/*
 * LCDCheck.h
 *
 *  Created on: 2020. 7. 20.
 *      Author: HiX
 */

#ifndef LCDCHECK_H_
#define LCDCHECK_H_

typedef enum{
	LCD_ON=0,
	LCD_OFF=1,
	LCD_NONE=2
}LCDState_t,*LCDState_p;

//volatile LCDStatus_t LCDState = LCD_NONE;

void setLCDState(LCDState_t);
LCDState_t getLCDState();

#endif /* LCDCHECK_H_ */
