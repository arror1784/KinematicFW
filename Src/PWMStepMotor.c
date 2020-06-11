/*
 * PWMStepMotor.c
 *
 *  Created on: 2019. 1. 25.
 *      Author: JeonSeungHwan
 */


#include "PWMStepMotor.h"
#include "StepMotorDriver.h"
#include "usart.h"
#include "common.h"
#include "stdlib.h"

uint8_t deviceCount = 0;
uint8_t endStopSignal = FALSE;
int8_t endStopCheckingBouncing = -1;
//extern HAL_UART_StateTypeDef huart2;

bool STMotorInitControl(void){
	//return A4988_ControlInit();
	return TRUE;
}

bool STMotorInitHandler(STMotorHandle_t* STMotorHandle, TIM_HandleTypeDef* Handle,
		HAL_TIM_ActiveChannel channel,IRQn_Type IRQn){

	STMotorHandle->motorHandler.timHandle = Handle;
	STMotorHandle->motorHandler.instance = Handle->Instance;
	STMotorHandle->motorHandler.timChaanel = channel;
	STMotorHandle->motorHandler.deviceNumber = deviceCount++;
	STMotorHandle->motorHandler.isActivate = FALSE;
	STMotorHandle->motorHandler.autoHoming = FALSE;
	STMotorHandle->motorHandler.timPrescaler = MOTOR_PRESCALER;
	STMotorHandle->motorHandler.IRQn = IRQn;
	STMotorHandle->motorHandler.port = MOTOR_END_GPIO_Port;
	STMotorHandle->motorHandler.pin = MOTOR_END_Pin;

	endStopSignal = FALSE;

	STMotorDeviceControl.SetEnableGPIO(STMotorHandle,TRUE);
	return TRUE;
}

bool STMotorInitParam(STMotorHandle_t* STMotorHandle, uint32_t accel,uint32_t decel,uint32_t maxSpeed,uint32_t minSpeed){

	STMotorHandle->motorParam.accel = (accel) ? accel : MOTOR_ACCEL;
	STMotorHandle->motorParam.accel_2 = (accel) ? accel : MOTOR_ACCEL;
	STMotorHandle->motorParam.decel = (decel) ? decel : MOTOR_DECEL;
	STMotorHandle->motorParam.decel_2 = (decel) ? decel : MOTOR_DECEL;
	STMotorHandle->motorParam.maxSpeed = (maxSpeed) ? maxSpeed : MOTOR_MAX_SPEED;
	STMotorHandle->motorParam.minSpeed = (minSpeed) ? minSpeed : MOTOR_MIN_SPEED;
	//STMotorHandle->motorParam.microStep = (microStep) ? microStep : MOTOR_MICRO_STEP;
	//STMotorHandle->motorParam.motorStep = (motorStep) ? motorStep : MOTOR_MOTOR_STEP;

	STMotorHandle->motorParam.curPosition = 0;
	STMotorHandle->motorParam.direction = DIR_UNKNOWN;
	STMotorHandle->motorParam.state = STATE_STOP;

	return TRUE;
}

uint32_t STMotorGetPosition(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.curPosition;
}
uint32_t STMotorGetCurSpeed(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.curSpeed;
}
uint32_t STMotorGetMaxSpeed(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.maxSpeed;
}
uint32_t STMotorGetMinSpeed(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.minSpeed;
}
uint8_t STMotorGetMicroStep(STMotorHandle_t* STMotorHandle){
	//return STMotorHandle->motorParam.microStep;
	return MOTOR_MICRO_STEP;
}
uint8_t STMotorGetMotorStep(STMotorHandle_t* STMotorHandle){
	//return STMotorHandle->motorParam.motorStep;
	return MOTOR_MOTOR_STEP;
}
uint8_t STMotorGetScrewPitch(STMotorDeviceControl_t* STMotorHandle){
	//return STMotorHandle->motorParam.screwPitch;
	return MOTOR_SCREW_PITCH;
}
uint16_t STMotorGetDeviceNum(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorHandler.deviceNumber;
}

uint16_t STMotorSetDeviceNum(STMotorHandle_t* STMotorHandle,uint16_t deviceNum){
	return STMotorHandle->motorHandler.deviceNumber = deviceNum;
}
uint32_t STMotorSetMaxSpeed(STMotorHandle_t* STMotorHandle, uint32_t maxSpeed,uint8_t mode){
	if(mode)
		STMotorHandle->motorParam.maxSpeed = maxSpeed;
	else
		STMotorHandle->motorParam.maxSpeed = maxSpeed;
	return maxSpeed;
//	return STMotorHandle->motorParam.maxSpeed = maxSpeed;
}
uint32_t STMotorSetMinSpeed(STMotorHandle_t* STMotorHandle, uint32_t minSpeed,uint8_t mode){
	if(mode)
		STMotorHandle->motorParam.minSpeed = minSpeed;
	else
		STMotorHandle->motorParam.minSpeed = minSpeed;
	return minSpeed;//	return STMotorHandle->motorParam.minSpeed = minSpeed;
}
uint32_t STMotorSetAccelSpeed(STMotorHandle_t* STMotorHandle, uint32_t accel,uint8_t mode){
	if(mode)
		STMotorHandle->motorParam.accel_2 = accel;
	else
		STMotorHandle->motorParam.accel = accel;
	return accel;//	return STMotorHandle->motorParam.accel = accel;
}
uint32_t STMotorSetDecelSpeed(STMotorHandle_t* STMotorHandle, uint32_t decel,uint8_t mode){
	if(mode)
		STMotorHandle->motorParam.decel_2 = decel;
	else
		STMotorHandle->motorParam.decel = decel;
	return decel;//	return STMotorHandle->motorParam.decel = decel;
}

uint32_t STMotorCalcAccelSpeed(STMotorHandle_t* STMotorHandle,uint32_t nStep,uint8_t mode){

	uint32_t accSteps;
	uint32_t decSteps;

	uint32_t minSpeed = STMotorHandle->motorParam.minSpeed;
	uint32_t maxSpeed = STMotorHandle->motorParam.maxSpeed;
	uint32_t acc = 0;
	uint32_t dec = 0;
	if(mode == 1){
		acc = STMotorHandle->motorParam.accel_2;
		dec = STMotorHandle->motorParam.decel_2;
	}else if(mode == 0){
		acc = STMotorHandle->motorParam.accel;
		dec = STMotorHandle->motorParam.decel;
	}

	STMotorHandle->motorParam.mode = mode;

	accSteps = (maxSpeed - minSpeed) * (maxSpeed + minSpeed);
	decSteps = accSteps;
	accSteps /= acc;
	accSteps /= 2;

	decSteps /= dec;
	decSteps /= 2;

	if((accSteps + decSteps) > nStep){

		decSteps = (dec * nStep) / (acc + dec);

		if(decSteps > 1){
			accSteps = decSteps - 1;
			if(accSteps == 0){
				accSteps = 1;
			}
		}else{
			accSteps = 0;
		}
		STMotorHandle->motorParam.endAccel = accSteps;
		STMotorHandle->motorParam.startDecel = decSteps;
	}else if(accSteps + decSteps == 0 ){
		STMotorHandle->motorParam.endAccel = 0;
		STMotorHandle->motorParam.startDecel = nStep + 5;
	}else{
		STMotorHandle->motorParam.endAccel = accSteps;
		STMotorHandle->motorParam.startDecel = nStep - decSteps - 1;
	}
	return 0;
}

uint32_t STMotorSetFreq(STMotorHandle_t* STMotorHandle,uint32_t freq){
	uint32_t sysFreq = HAL_RCC_GetSysClockFreq();
	uint32_t period = (sysFreq/ (STMotorHandle->motorHandler.timPrescaler * (uint32_t)freq)) - 1;

	STMotorDeviceControl.SetEnableGPIO(STMotorHandle,TRUE);
	__HAL_TIM_SET_AUTORELOAD(STMotorHandle->motorHandler.timHandle, period);
	__HAL_TIM_SET_COMPARE(STMotorHandle->motorHandler.timHandle, STMotorHandle->motorHandler.timChaanel, period >> 1);
	HAL_TIM_PWM_Start_IT(STMotorHandle->motorHandler.timHandle, STMotorHandle->motorHandler.timChaanel);

	return 0;
}

uint32_t STMotorStopFreq(STMotorHandle_t* STMotorHandle){
	HAL_TIM_PWM_Stop_IT(STMotorHandle->motorHandler.timHandle,STMotorHandle->motorHandler.timChaanel);
	return 0;
}

uint32_t STMotorGoStep(STMotorHandle_t* STMotorHandle,int32_t step,uint8_t mode){

	if(STMotorHandle->motorHandler.isActivate == FALSE && STMotorHandle->motorHandler.autoHoming == FALSE){
		STMotorHandle->motorHandler.isActivate = TRUE;
		if(step == 0){
			STMotorHandle->motorHandler.isActivate = FALSE;
			return 1;
		}else if(step < 0) {
			STMotorHandle->motorParam.targetStep = (-step) - 1;
			STMotorHandle->motorParam.direction = DIR_BACKWORD;
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_BACKWORD);
		}else{
			STMotorHandle->motorParam.targetStep = step - 1;
			STMotorHandle->motorParam.direction = DIR_FORWORD;
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_FORWORD);
		}

		STMotorCalcAccelSpeed(STMotorHandle,STMotorHandle->motorParam.targetStep,mode);

		STMotorMoveStart(STMotorHandle);

		return 0;
	}else{
		return 1;
	}
}

uint32_t STMotorMoveStart(STMotorHandle_t* STMotorHandle){

	STMotorHandle->motorParam.accu = 0;
	STMotorHandle->motorParam.nStep = 0;

	if(STMotorHandle->motorParam.endAccel != 0){
		STMotorHandle->motorParam.state = STATE_ACCEL;
	}else{
		STMotorHandle->motorParam.state = STATE_STAND;
	}
	if(STMotorHandle->motorParam.minSpeed > STMotorHandle->motorParam.maxSpeed)
		STMotorHandle->motorParam.minSpeed = STMotorHandle->motorParam.maxSpeed;

	STMotorHandle->motorParam.curSpeed = STMotorHandle->motorParam.minSpeed;
	STMotorSetFreq(STMotorHandle,STMotorHandle->motorParam.minSpeed);

	return 0;
}

uint32_t STMotorGoSpeed(STMotorHandle_t* STMotorHandle,int32_t speed,uint16_t timeOut){

	if(STMotorHandle->motorHandler.isActivate == FALSE && STMotorHandle->motorHandler.autoHoming == FALSE){
		STMotorHandle->motorHandler.isActivate = TRUE;
		if(speed == 0){
			STMotorHandle->motorHandler.isActivate = FALSE;
			return 0;
		}else if(speed < 0) {
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_BACKWORD);
		}else{
			STMotorDeviceControl.SetDirectionGPIO(STMotorHandle,DIR_FORWORD);
		}
		STMotorHandle->motorParam.accu = 0;
		STMotorHandle->motorParam.endAccel = 0;
		STMotorHandle->motorParam.startDecel = 0;
		STMotorHandle->motorParam.targetStep = 0;
		STMotorHandle->motorParam.nStep = 0;
		STMotorHandle->motorParam.curSpeed = speed;

		STMotorHandle->motorParam.state = STATE_INFINITE;
		STMotorSetFreq(STMotorHandle,abs(speed));
		return 0;
	}else{
		return 1;
	}

}

uint32_t STMotorAutoHome(STMotorHandle_t* STMotorHandle,int32_t speed){
	if(STMotorHandle->motorHandler.isActivate == FALSE && STMotorHandle->motorHandler.autoHoming == FALSE){
		if(HAL_GPIO_ReadPin(STMotorHandle->motorHandler.port,STMotorHandle->motorHandler.pin)){
			__HAL_GPIO_EXTI_CLEAR_IT(STMotorHandle->motorHandler.pin);
//			HAL_NVIC_EnableIRQ(STMotorHandle->motorHandler.IRQn);
			endStopSignal = TRUE;
			if(speed == 0){
				if(STMotorHandle->motorParam.minSpeed > STMotorHandle->motorParam.maxSpeed)
					STMotorHandle->motorParam.minSpeed = STMotorHandle->motorParam.maxSpeed;

				STMotorGoSpeed(STMotorHandle,STMotorHandle->motorParam.minSpeed,0);
			}else{
				STMotorGoSpeed(STMotorHandle,speed,0);
			}
			STMotorHandle->motorHandler.autoHoming = TRUE;
			return 0;
		}else{
			STMotorHandle->motorParam.state = STATE_STOP;
			STMotorHandle->motorParam.nStep = 0;
			STMotorHandle->motorParam.targetStep = 0;
			STMotorHandle->motorHandler.isActivate = FALSE;
			STMotorSetHome(STMotorHandle);
			STMotorDeviceControl.FinishCallBack(STMotorHandle);
			return 1;
		}
	}else{
		while(HAL_UART_Transmit(&huart3,(uint8_t*)"is activate\r\n",13,1000) != HAL_OK);
		return 2;
	}

}

//uint32_t STMotorGoMilli(STMotorHandle_t* STMotorHandle,double milli){
//	return STMotorGoStep(STMotorHandle, STMotorCalcMilliToStep(milli));
//}
uint32_t STMotorGoMicro(STMotorHandle_t* STMotorHandle,int32_t micro,uint8_t mode){
	return STMotorGoStep(STMotorHandle, STMotorCalcMicroToStep(micro),mode);
}
uint32_t STMotorGoRotation(STMotorHandle_t* STMotorHandle,double rotation,uint8_t mode){
	return STMotorGoStep(STMotorHandle, STMotorCalcRotationToStep(rotation),mode);
}

uint32_t STMotorHardStop(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.state = STATE_HARDSTOP;
}
uint32_t STMotorSoftStop(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorParam.state = STATE_SOFTSTOP;
}

uint32_t STMotorWaitingActivate(STMotorHandle_t* STMotorHandle,uint32_t timeOut){
	uint32_t startMilli = HAL_GetTick();
	while(STMotorHandle->motorHandler.isActivate == TRUE && STMotorHandle->motorHandler.autoHoming == TRUE){
		if(timeOut == 0){
			continue;
		}else if(HAL_GetTick() > startMilli + timeOut){
			STMotorHandle->motorHandler.isActivate = FALSE;
			STMotorHardStop(STMotorHandle);
			break;
		}
	}
	return 0;
}
uint8_t STMotorIsActivate(STMotorHandle_t* STMotorHandle){
	return STMotorHandle->motorHandler.isActivate;
}

uint32_t STMotorGoHome(STMotorHandle_t* STMotorHandle){

	int32_t step = -STMotorHandle->motorParam.curPosition;
	if(STMotorGoStep(STMotorHandle,step,0) == 1){
		STMotorDeviceControl.FinishCallBack(STMotorHandle);
	}
	return 0;
}
uint32_t STMotorSetHome(STMotorHandle_t* STMotorHandle){

	STMotorHandle->motorParam.curPosition = 0;
	return 0;
}

double STMotorCalcStepToRotation(uint32_t nStep){
	return nStep/MOTOR_MICRO_STEP * MOTOR_MOTOR_STEP;
}
int32_t STMotorCalcRotationToStep(double rotation){
	return (double)(MOTOR_MICRO_STEP * MOTOR_MOTOR_STEP) * rotation;
}

int32_t STMotorCalcRotationToMicro(double rotation){
	return rotation * MOTOR_SCREW_PITCH;
}
double STMotorCalcMicroToRotation(int32_t micro){
	return micro / (double)MOTOR_SCREW_PITCH;
}

int32_t STMotorCalcStepToMilli(uint32_t nStep){
	return nStep/(MOTOR_MICRO_STEP * MOTOR_MOTOR_STEP) * MOTOR_SCREW_PITCH;
}
int32_t STMotorCalcMicroToStep(int32_t micro){
	return micro / (((double)MOTOR_SCREW_PITCH) / ((double)MOTOR_MICRO_STEP * (double)MOTOR_MOTOR_STEP));
}

uint32_t STMotorPWMPulseInterruptHandle(STMotorHandle_t* STMotorHandle){

	uint32_t relStep = STMotorHandle->motorParam.nStep++;
	uint32_t targetStep = STMotorHandle->motorParam.targetStep;
	uint32_t speed = STMotorHandle->motorParam.curSpeed;

	uint32_t accel;
	uint32_t decel;

	if(STMotorHandle->motorParam.mode){
		accel = STMotorHandle->motorParam.accel_2 << 16;
		decel = STMotorHandle->motorParam.decel_2 << 16;
	}else{
		accel = STMotorHandle->motorParam.accel << 16;
		decel = STMotorHandle->motorParam.decel << 16;

	}

	bool speedUpdated = FALSE;

	switch(STMotorHandle->motorParam.state){

		case STATE_INFINITE:
			break;
		case STATE_ACCEL:
			if(relStep >= targetStep - 1){
				STMotorHandle->motorParam.state = STATE_STOP;
			}else if(relStep >= STMotorHandle->motorParam.endAccel){
				STMotorHandle->motorParam.state = STATE_STAND;
			}else{
				if(speed < STMotorHandle->motorParam.minSpeed)
					speed = STMotorHandle->motorParam.minSpeed;
				STMotorHandle->motorParam.accu += accel / speed;
				while (STMotorHandle->motorParam.accu >= (0X10000L))
				{
					STMotorHandle->motorParam.accu -= (0X10000L);
					speed += 1;
					speedUpdated = TRUE;
				}
				if(speedUpdated == TRUE){
					speedUpdated = FALSE;
					if(speed > STMotorHandle->motorParam.maxSpeed)
						speed = STMotorHandle->motorParam.maxSpeed;
					STMotorHandle->motorParam.curSpeed = speed;
					STMotorSetFreq(STMotorHandle,speed);
				}
			}
			break;
		case STATE_STAND:
			if( relStep >= STMotorHandle->motorParam.startDecel - 1){
				STMotorHandle->motorParam.accu = 0;
				STMotorHandle->motorParam.state = STATE_DECEL;
			}else if(relStep < STMotorHandle->motorParam.endAccel){
				STMotorHandle->motorParam.accu = 0;
				STMotorHandle->motorParam.state = STATE_ACCEL;
			}else if(relStep >= STMotorHandle->motorParam.targetStep - 1){
				STMotorHandle->motorParam.accu = 0;
				STMotorHandle->motorParam.state = STATE_STOP;
			}
			break;
		case STATE_SOFTSTOP:
		case STATE_DECEL:
			if(relStep >= targetStep - 1){
				STMotorHandle->motorParam.state = STATE_STOP;
			}/*else if(relStep <= STMotorHandle->motorParam.startDecel){
				STMotorHandle->motorParam.state = STATE_STAND;
			}*/else if(STMotorHandle->motorParam.state == STATE_HARDSTOP){
				STMotorHandle->motorParam.state = STATE_STOP;
			}else{

				speedUpdated = FALSE;
		        if (speed == 0) speed =1;

				STMotorHandle->motorParam.accu += decel / speed;
				while (STMotorHandle->motorParam.accu >= (0X10000L))
				{
					STMotorHandle->motorParam.accu -= (0X10000L);
					if (speed > 1){
						speed -=1;
					}
					speedUpdated = TRUE;
				}
				if(speedUpdated == TRUE){
					speedUpdated = FALSE;
					if(speed < STMotorHandle->motorParam.minSpeed)
						speed = STMotorHandle->motorParam.minSpeed;
					STMotorHandle->motorParam.curSpeed = speed;
					STMotorSetFreq(STMotorHandle,speed);
				}
			}
			break;
		case STATE_HARDSTOP:
		case STATE_STOP:
			STMotorHandle->motorParam.curPosition += (relStep + 1) * ((STMotorHandle->motorParam.direction) ? 1 : -1);
			STMotorHandle->motorParam.nStep = 0;
			STMotorHandle->motorParam.targetStep = 0;
			STMotorHandle->motorParam.state = STATE_STOP;
			STMotorDeviceControl.SetEnableGPIO(STMotorHandle,TRUE);
			STMotorStopFreq(STMotorHandle);
			STMotorHandle->motorHandler.isActivate = FALSE;
			STMotorDeviceControl.FinishCallBack(STMotorHandle);

			break;
		case STATE_FINISH_AUTO_HOME:
//			if(!(--STMotorHandle->motorParam.targetStep)){
				STMotorHandle->motorParam.state = STATE_STOP;
				STMotorHandle->motorParam.nStep = 0;
				STMotorHandle->motorParam.targetStep = 0;
				STMotorHandle->motorHandler.isActivate = FALSE;
				STMotorDeviceControl.SetEnableGPIO(STMotorHandle,TRUE);
				STMotorStopFreq(STMotorHandle);

				endStopCheckingBouncing = STMotorHandle->motorHandler.deviceNumber;

				STMotorSetHome(STMotorHandle);
//			}
		default:
			break;
	}

	return 0;
}

uint32_t STMotorEXTInterruptHandle(STMotorHandle_t* STMotorHandle){
//	HAL_NVIC_DisableIRQ(STMotorHandle->motorHandler.IRQn);
//	endStopSignal = FALSE;
//	STMotorHardStop(STMotorHandle);

//	STMotorHandle->motorParam.targetStep = (MOTOR_MOTOR_STEP * MOTOR_MICRO_STEP) / 2;
	STMotorHandle->motorParam.state = STATE_FINISH_AUTO_HOME;

//	STMotorHardStop(STMotorHandle);

//	STMotorSetHome(STMotorHandle);
	return 0;
}

void STMotorEXTInterruptENDBouncing(STMotorHandle_t* STMotorHandle){
	static uint32_t TK = 0;
	int currentTime = 0;
	static int F = 0;

	if(!F){

		TK = HAL_GetTick();
		F=1;

	}else{

		currentTime = HAL_GetTick();

		if((currentTime - TK) >= 1000){
			if(!HAL_GPIO_ReadPin(STMotorHandle->motorHandler.port,STMotorHandle->motorHandler.pin)){
				endStopSignal = FALSE;
				endStopCheckingBouncing = -1;
				F=0;
				STMotorDeviceControl.FinishCallBack(STMotorHandle);
				STMotorHandle->motorHandler.autoHoming = FALSE;

				while(HAL_UART_Transmit(&huart3,"sucess\r\n",8,1000) != HAL_OK);
			}else{
				STMotorHandle->motorHandler.autoHoming = FALSE;
				STMotorGoSpeed(STMotorHandle,STMotorHandle->motorParam.curSpeed,0);
				STMotorHandle->motorHandler.autoHoming = TRUE;

				while(HAL_UART_Transmit(&huart3,"fail\r\n",6,1000) != HAL_OK);
				endStopCheckingBouncing = -1;
				F=0;
			}
		}

	}
}



