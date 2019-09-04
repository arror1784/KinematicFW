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
#include "neoPixel.h"

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

void G01(HGCodeDataControl_t* temp);
void G02(HGCodeDataControl_t* temp);
void G04(HGCodeDataControl_t* temp);
void G05(HGCodeDataControl_t* temp);
void G27(HGCodeDataControl_t* temp);
void G28(HGCodeDataControl_t* temp);
void G30(HGCodeDataControl_t* temp);
void G31(HGCodeDataControl_t* temp);
void G99(HGCodeDataControl_t* temp);

void H01(HGCodeDataControl_t* temp);
void H05(HGCodeDataControl_t* temp);
void H10(HGCodeDataControl_t* temp);
void H11(HGCodeDataControl_t* temp);
void H20(HGCodeDataControl_t* temp);
void H21(HGCodeDataControl_t* temp);
void H30(HGCodeDataControl_t* temp);
void H31(HGCodeDataControl_t* temp);
void H32(HGCodeDataControl_t* temp);
void H33(HGCodeDataControl_t* temp);

void H40(HGCodeDataControl_t* temp);
void H41(HGCodeDataControl_t* temp);
void H42(HGCodeDataControl_t* temp);
void H43(HGCodeDataControl_t* temp);
void H50(HGCodeDataControl_t* temp,WS2812BControl_p neoPixel_P);
void H51(HGCodeDataControl_t* temp,WS2812BControl_p neoPixel_P);
void H60(HGCodeDataControl_t* temp);

void H100(HGCodeDataControl_t* temp); // check

void sendResponse(uint8_t bed,uint8_t commnad,uint8_t response);

#endif /* HGC_INC_HGCODEFUNCTION_H_ */
