/*
 * HGCodeFunction.h
 *
 *  Created on: 2019. 2. 15.
 *      Author: JeonSeungHwan
 */

#ifndef HGC_INC_HGCODEFUNCTION_H_
#define HGC_INC_HGCODEFUNCTION_H_

#include "HGCodeParser.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "tim.h"
#include "PWMStepMotor.h"
#include "StepMotorDriver.h"

/*typedef struct{

	void (*G0)(void);
	void (*G1)(void);
	void (*G4)(void);
	void (*G5)(void);
	void (*G27)(void);
	void (*G28)(void);
	void (*G30)(void);
	void (*G31)(void);
	void (*G99)(void);

	void (*H0)(void);
	void (*H5)(void);
	void (*H10)(void);
	void (*H11)(void);

}HGCodeFunction_t;*/

void startHGCode(TIM_HandleTypeDef* timHandler,UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle);

void G01();
void G02();
void G04();
void G05();
void G27();
void G28();
void G30();
void G31();
void G99();

void H01();
void H05();
void H10();
void H11();
void H20();
void H21();
void H30();
void H31();
void H32();
void H33();
void H50();


void H100();

#endif /* HGC_INC_HGCODEFUNCTION_H_ */
