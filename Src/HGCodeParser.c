/*
 * HGCodeParser.c
 *
 *  Created on: 2019. 2. 1.
 *      Author: JeonSeungHwan
 */

#include "HGCodeParser.h"
#include "usart.h"
#include "string.h"

uint8_t HGCodeBuffer[MAX_HGCODE_BUFFER] = {0};
HGCodeControl_t HGCodeControl = {0};
uint8_t buff[20] = {0};

void HGCodeInit(UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle){
	HGCodeControl.HGCodeUartHandle = HGCodeUsartHandle;
	HGCodeControl.HGCodeDmaHandle = HGCodeDmaHandle;
	HGCodeControl.HGCodeBufferControl.GHCodeHuffer = HGCodeBuffer;
	HGCodeControl.HGCodeBufferControl.bufferSize = MAX_HGCODE_BUFFER;

	HGCodeControl.commandCount = 0;
	HGCodeControl.HGCodeBufferControl.front = 0;
	HGCodeControl.HGCodeBufferControl.rear = 0;

	HGCodeControl.dataRear = 0;
	HGCodeControl.dataFront = 0;

	memset(HGCodeControl.HGCodeDataControl,0x00,sizeof(CommandFormat_t)*MAX_COMMAND);
}

	//void HGCodeDMASetBuffer(int8_t *,uint16_t);

void HGCodeDMAStart(void){
	HAL_UART_Receive_DMA(HGCodeControl.HGCodeUartHandle,HGCodeBuffer,(int)MAX_HGCODE_BUFFER);
}

int8_t HGCodeCheckCommandBuffer(void){
	if(HGCodeControl.HGCodeBufferControl.rear == MAX_HGCODE_BUFFER - __HAL_DMA_GET_COUNTER(HGCodeControl.HGCodeDmaHandle)){
		return 0;
	}else{
		return 1;
	}
}

int8_t HGCodeCheckDataBuffer(void){
	if(HGCodeGetCommandCount() > 0){
		return 1;
	}else{
		return 0;
	}
}

void HGCodeDecodeCommand(void){

	uint32_t rear = HGCodeControl.HGCodeBufferControl.rear;
	uint32_t front = MAX_HGCODE_BUFFER - __HAL_DMA_GET_COUNTER(HGCodeControl.HGCodeDmaHandle);
	uint32_t receiveDataSize;
	int index = rear;
//	int data;
	uint8_t checksum = 0;
	CommandFormat_t temp = {0};
//	int test_int;
//	double test_db;

	if( rear < front){
		receiveDataSize = front - rear;
	}else if(rear > front){
		receiveDataSize = front + (MAX_HGCODE_BUFFER - rear);
	}else{
		return;
	}

	if(receiveDataSize < 25){
		return;
	}else{
		for(int i = receiveDataSize; i > 0; i--,index = (index + 1) % MAX_HGCODE_BUFFER){
			if(HGCodeBuffer[index] == 0x02 && HGCodeBuffer[(index + 24) % MAX_HGCODE_BUFFER] == 0x03){
				checksum = 0;
				for(int j = 1; j < 23; j++){
					checksum += HGCodeBuffer[(index + j) % MAX_HGCODE_BUFFER];
				}
				if(checksum != HGCodeBuffer[(index + 23) % MAX_HGCODE_BUFFER]){
					HAL_UART_Transmit_IT(&huart3,"transmit error\r\n",16);
//					HAL_UART_Transmit_IT(&huart2,response,8);
					sendResponse(0,100,0);
					break;
				}else{
					temp.G = (uint8_t)HGCodeBuffer[(index + 1) % MAX_HGCODE_BUFFER];
					temp.H = (uint8_t)HGCodeBuffer[(index + 2) % MAX_HGCODE_BUFFER];

					temp.A.ch[0] = HGCodeBuffer[(index + 3) % MAX_HGCODE_BUFFER];
					temp.A.ch[1] = HGCodeBuffer[(index + 4) % MAX_HGCODE_BUFFER];
					temp.A.ch[2] = HGCodeBuffer[(index + 5) % MAX_HGCODE_BUFFER];
					temp.A.ch[3] = HGCodeBuffer[(index + 6) % MAX_HGCODE_BUFFER];
	//				temp.A.ch[4] = HGCodeBuffer[(index + 7) % MAX_HGCODE_BUFFER];
	//				temp.A.ch[5] = HGCodeBuffer[(index + 8) % MAX_HGCODE_BUFFER];
	//				temp.A.ch[6] = HGCodeBuffer[(index + 9) % MAX_HGCODE_BUFFER];
	//				temp.A.ch[7] = HGCodeBuffer[(index + 10) % MAX_HGCODE_BUFFER];

					temp.B.ch[0] = HGCodeBuffer[(index + 7) % MAX_HGCODE_BUFFER];
					temp.B.ch[1] = HGCodeBuffer[(index + 8) % MAX_HGCODE_BUFFER];
					temp.B.ch[2] = HGCodeBuffer[(index + 9) % MAX_HGCODE_BUFFER];
					temp.B.ch[3] = HGCodeBuffer[(index + 10) % MAX_HGCODE_BUFFER];
	//				temp.B.ch[4] = HGCodeBuffer[(index + 15) % MAX_HGCODE_BUFFER];
	//				temp.B.ch[5] = HGCodeBuffer[(index + 16) % MAX_HGCODE_BUFFER];
	//				temp.B.ch[6] = HGCodeBuffer[(index + 17) % MAX_HGCODE_BUFFER];
	//				temp.B.ch[7] = HGCodeBuffer[(index + 18) % MAX_HGCODE_BUFFER];

					temp.C.ch[0] = HGCodeBuffer[(index + 11) % MAX_HGCODE_BUFFER];
					temp.C.ch[1] = HGCodeBuffer[(index + 12) % MAX_HGCODE_BUFFER];
					temp.C.ch[2] = HGCodeBuffer[(index + 13) % MAX_HGCODE_BUFFER];
					temp.C.ch[3] = HGCodeBuffer[(index + 14) % MAX_HGCODE_BUFFER];
	//				temp.C.ch[4] = HGCodeBuffer[(index + 23) % MAX_HGCODE_BUFFER];
	//				temp.C.ch[5] = HGCodeBuffer[(index + 24) % MAX_HGCODE_BUFFER];
	//				temp.C.ch[6] = HGCodeBuffer[(index + 25) % MAX_HGCODE_BUFFER];
	//				temp.C.ch[7] = HGCodeBuffer[(index + 26) % MAX_HGCODE_BUFFER];

					temp.M.ch[0] = HGCodeBuffer[(index + 15) % MAX_HGCODE_BUFFER];
					temp.M.ch[1] = HGCodeBuffer[(index + 16) % MAX_HGCODE_BUFFER];
					temp.M.ch[2] = HGCodeBuffer[(index + 17) % MAX_HGCODE_BUFFER];
					temp.M.ch[3] = HGCodeBuffer[(index + 18) % MAX_HGCODE_BUFFER];
	//				temp.M.ch[4] = HGCodeBuffer[(index + 31) % MAX_HGCODE_BUFFER];
	//				temp.M.ch[5] = HGCodeBuffer[(index + 32) % MAX_HGCODE_BUFFER];
	//				temp.M.ch[6] = HGCodeBuffer[(index + 33) % MAX_HGCODE_BUFFER];
	//				temp.M.ch[7] = HGCodeBuffer[(index + 34) % MAX_HGCODE_BUFFER];

					temp.P.ch[0] = HGCodeBuffer[(index + 19) % MAX_HGCODE_BUFFER];
					temp.P.ch[1] = HGCodeBuffer[(index + 20) % MAX_HGCODE_BUFFER];
					temp.P.ch[2] = HGCodeBuffer[(index + 21) % MAX_HGCODE_BUFFER];
					temp.P.ch[3] = HGCodeBuffer[(index + 22) % MAX_HGCODE_BUFFER];
	//				temp.P.ch[4] = HGCodeBuffer[(index + 39) % MAX_HGCODE_BUFFER];
	//				temp.P.ch[5] = HGCodeBuffer[(index + 40) % MAX_HGCODE_BUFFER];
	//				temp.P.ch[6] = HGCodeBuffer[(index + 41) % MAX_HGCODE_BUFFER];
	//				temp.P.ch[7] = HGCodeBuffer[(index + 42) % MAX_HGCODE_BUFFER];

	//				HGCodePutMainCommand(temp.G,HGCODE_G);
	//				HGCodePutMainCommand(temp.H,HGCODE_H);
	//				HGCodePutSubCommand(temp.A.db / 1000.0f,HGCODE_A);
	//				HGCodePutSubCommand(temp.B.db / 1000.0f,HGCODE_B);
	//				HGCodePutSubCommand(temp.C.db / 1000.0f,HGCODE_C);
	//				HGCodePutSubCommand(temp.M.db / 1000.0f,HGCODE_M);
	//				HGCodePutSubCommand(temp.P.db / 1000.0f,HGCODE_P);

					HGCodePutMainCommand(temp.G,HGCODE_G);
					HGCodePutMainCommand(temp.H,HGCODE_H);
					HGCodePutSubCommand(temp.A.int32, HGCODE_A);
					HGCodePutSubCommand(temp.B.int32, HGCODE_B);
					HGCodePutSubCommand(temp.C.int32, HGCODE_C);
					HGCodePutSubCommand(temp.M.int32, HGCODE_M);
					HGCodePutSubCommand(temp.P.int32, HGCODE_P);

					HGCodeControl.dataFront = (HGCodeControl.dataFront + 1) % MAX_COMMAND;
					HGCodeControl.commandCount++;
					break;
				}
			}
		}
		HGCodeControl.HGCodeBufferControl.rear = (index + 25) % MAX_HGCODE_BUFFER;
	}
}

uint16_t HGCodeGetCommandCount (void){
	return HGCodeControl.commandCount;
}

HGCodeDataControl_t* HGCodeGetCommandData(void){

	uint8_t rear = HGCodeControl.dataRear;

	if(rear == HGCodeControl.dataFront)
		return 0;
	if(HGCodeControl.commandCount == 0)
		return 0;
	HGCodeControl.dataRear = (HGCodeControl.dataRear + 1) % MAX_COMMAND;

	HGCodeControl.commandCount -= 1;

	return &HGCodeControl.HGCodeDataControl[rear];
}

void HGCodePutMainCommand(uint8_t data, uint8_t flag){

	switch(flag){
	case HGCODE_G:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeCommand.G = data;
		break;
	case HGCODE_H:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeCommand.H = data;
		break;
	}
}
void HGCodePutSubCommand(int32_t data, uint8_t flag){

	switch(flag){
	case HGCODE_A:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeParameter.A = data;
		break;
	case HGCODE_B:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeParameter.B = data;
		break;
	case HGCODE_C:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeParameter.C = data;
		break;
	case HGCODE_P:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeParameter.P = data;
		break;
	case HGCODE_M:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeParameter.M = data;
		break;
	case HGCODE_CHECKSUM:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeParameter.checkSum = data;
		break;
	}
}
void DecodeTimerInterruptHandler(void){
	if(HGCodeCheckCommandBuffer() == 1){
		HGCodeDecodeCommand();
	}
}

