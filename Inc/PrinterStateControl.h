/*
 * PrinterStateControl.h
 *
 *  Created on: 2020. 5. 23.
 *      Author: hix
 */

#ifndef PRINTERSTATECONTROL_H_
#define PRINTERSTATECONTROL_H_

#include "common.h"

typedef enum{
	POWER_ON = 1,
	POWER_OFF,
}PowerState_t;

void setPowerState(PowerState_t);
PowerState_t getPowerState();

void powerOn();
void powerOff();

void softPowerOff();

void controlPowerStateBTN(BtnChannel_t);

#endif /* PRINTERSTATECONTROL_H_ */
