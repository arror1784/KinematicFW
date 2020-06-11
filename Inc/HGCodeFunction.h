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

void G01(HGCodeDataControl_t* temp); // go micro relative
void G02(HGCodeDataControl_t* temp); // go absolute
void G03(HGCodeDataControl_t* temp); // go step
void G04(HGCodeDataControl_t* temp); // null
void G05(HGCodeDataControl_t* temp); // null
void G27(HGCodeDataControl_t* temp); // go home
void G28(HGCodeDataControl_t* temp); // set home
void G30(HGCodeDataControl_t* temp); // null
void G31(HGCodeDataControl_t* temp); // null
void G99(HGCodeDataControl_t* temp); // null

void H01(HGCodeDataControl_t* temp); // null
void H05(HGCodeDataControl_t* temp); // null
void H10(HGCodeDataControl_t* temp); // UV LED off
void H11(HGCodeDataControl_t* temp); // UV LED on
void H12(HGCodeDataControl_t* temp); // SET UV LED PWM
void H20(HGCodeDataControl_t* temp); // UV COOLER OFF
void H21(HGCodeDataControl_t* temp); // UV COOLER ON
void H22(HGCodeDataControl_t* temp); // SET UV COOLER PWM

void H30(HGCodeDataControl_t* temp); // set MAX SPEED
void H31(HGCodeDataControl_t* temp); // set MIN SPEED
void H32(HGCodeDataControl_t* temp); // set ACCEL SPEED
void H33(HGCodeDataControl_t* temp); // set DECEL SPEED

void H40(HGCodeDataControl_t* temp); //null
void H41(HGCodeDataControl_t* temp); //null
void H42(HGCodeDataControl_t* temp); //null
void H43(HGCodeDataControl_t* temp); //null
void H44(HGCodeDataControl_t* temp); //null
void H50(HGCodeDataControl_t* temp,WS2812BControl_p neoPixel_P); // neopixel
void H51(HGCodeDataControl_t* temp,WS2812BControl_p neoPixel_P); // blank neopixel
void H60(HGCodeDataControl_t* temp); // null

void H100(HGCodeDataControl_t* temp); // MCU power check
void H101(HGCodeDataControl_t* temp); // LCD power check

void H200(HGCodeDataControl_t* temp); // receive power off siganl

void sendResponse(uint8_t bed,uint8_t commnad,uint8_t response);

//102 short
//103 long
//104 long long

#endif /* HGC_INC_HGCODEFUNCTION_H_ */
