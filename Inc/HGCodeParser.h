/*
 * HGCodeParser.h
 *
 *  Created on: 2019. 2. 1.
 *      Author: JeonSeungHwan
 */

#ifndef HGC_INC_HGCODEPARSER_H_
#define HGC_INC_HGCODEPARSER_H_

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "usart.h"

#include "common.h"

#define MAX_HGCODE_BUFFER 700
#define MAX_COMMAND 20

#define HGCODE_G		0
#define HGCODE_H		1
#define HGCODE_A		2
#define HGCODE_B		3
#define HGCODE_C		4
#define HGCODE_P		5
#define HGCODE_CHECKSUM	6
#define HGCODE_M		7

typedef struct {

	int16_t G;
	int16_t H;

}HGCodeCommand_t;

typedef struct {

	int32_t A;			//A bed
	int32_t B;			//B bed
	int32_t C;			//C bed
	int32_t P;			//projector
	int32_t checkSum; 	// checkSum
	int32_t M;			//milli

}HGCodeParameter_t;

typedef union{

    int32_t int32;
    char ch[4];

}Int2char;

typedef struct{

    uint8_t G;
    uint8_t H;
    Int2char A;
    Int2char B;
    Int2char C;
    Int2char M;
    Int2char P;

}CommandFormat_t,*CommandFormat_p;

typedef struct{
	uint8_t bed;
	uint8_t command;
	uint8_t response;

}ResponseData_t,*ResponseData_p;

typedef struct{

	uint8_t* GHCodeHuffer;
	uint32_t bufferSize;

	uint32_t front;
	uint32_t rear;

}HGCodeBufferControl_t;

typedef struct{

	HGCodeCommand_t HGCodeCommand;
	HGCodeParameter_t HGCodeParameter;

}HGCodeDataControl_t;

typedef struct{

	UART_HandleTypeDef* HGCodeUartHandle;
	DMA_HandleTypeDef* HGCodeDmaHandle;
	HGCodeBufferControl_t HGCodeBufferControl;
	HGCodeDataControl_t HGCodeDataControl[MAX_COMMAND];
//	commandFormat_t HGCodeDataControl[MAX_COMMAND];

	uint8_t commandCount;
	uint8_t dataFront;
	uint8_t dataRear;

}HGCodeControl_t;

typedef struct{

	uint8_t isdecoding;
	uint8_t isNewCommand;
	uint8_t checkSum;

}HGCodeState_t;

void startHGCode(TIM_HandleTypeDef* timHandler,UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle);

void HGCodeInit(UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle);
//void HGCodeDMASetBuffer(int8_t *,uint16_t);

void HGCodeDMAStart(void);
void HGCodeDMAPause(void);
void HGCodeDMAResume(void);

bool HGCodeCheckCommandBuffer(void);
bool HGCodeCheckDataBuffer(void);

void HGCodeDecodeCommand(void);

uint16_t HGCodeGetCommandCount(void);

HGCodeDataControl_t* HGCodeGetCommandData(void);

void HGCodePutMainCommand(uint8_t,uint8_t);
void HGCodePutSubCommand(int32_t, uint8_t);

double HGCodeCharToDouble(uint8_t* ,int size);
int16_t HGCodeCharToInt(uint8_t* ,int size);

//(index+1) % 배열의 사이즈

void DecodeTimerInterruptHandler(void);

#endif /* HGC_INC_HGCODEPARSER_H_ */
