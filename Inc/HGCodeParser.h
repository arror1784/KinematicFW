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
#include <usart.h>

#define MAX_HGCODE_BUFFER 200
#define MAX_COMMAND 10

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

typedef union{

    float db;
    char ch[4];

}double2char;

typedef struct{

    uint8_t G;
    uint8_t H;
    double2char A;
    double2char B;
    double2char C;
    double2char M;
    double2char P;
    double2char CheckSum;

}commandFormat_t,*commandFormat_P;

typedef struct {

	double A;			//A bed
	double B;			//B bed
	double C;			//C bed
	double P;			//projector
	double checkSum; 	// checkSum
	double M;			//milli

}HGCodeParameter_t;

typedef struct{

	uint8_t* GHCodeHuffer;
	uint16_t bufferSize;

	uint8_t front;
	uint8_t rear;

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

typedef struct{

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

}HGCodeFunction_t;

void HGCodeInit(UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle);
//void HGCodeDMASetBuffer(int8_t *,uint16_t);

void HGCodeDMAStart(void);

int8_t HGCodeCheckCommandBuffer(void);
int8_t HGCodeCheckDataBuffer(void);

void HGCodeDecodeCommand(void);

uint16_t HGCodeGetCommandCount(void);

HGCodeDataControl_t* HGCodeGetCommandData(void);

void HGCodePutData(double, uint8_t);

double HGCodeCharToDouble(uint8_t* ,int size);
int16_t HGCodeCharToInt(uint8_t* ,int size);

//(index+1) % 배열의 사이즈

void DecodeTimerInterruptHandler(void);

#endif /* HGC_INC_HGCODEPARSER_H_ */
