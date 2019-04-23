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

void HGCodeInit(UART_HandleTypeDef* HGCodeUsartHandle,DMA_HandleTypeDef* HGCodeDmaHandle){
	HGCodeControl.HGCodeUartHandle =HGCodeUsartHandle;
	HGCodeControl.HGCodeDmaHandle = HGCodeDmaHandle;
	HGCodeControl.HGCodeBufferControl.GHCodeHuffer = HGCodeBuffer;
	HGCodeControl.HGCodeBufferControl.bufferSize = MAX_HGCODE_BUFFER;

	HGCodeControl.commandCount = 0;
	HGCodeControl.HGCodeBufferControl.front = 0;
	HGCodeControl.HGCodeBufferControl.rear = 0;

	HGCodeControl.dataRear = 0;
	HGCodeControl.dataFront = 0;

	/*for(int i = 0 ; i < MAX_COMMAND;i++){
		HGCodeControl.HGCodeDataControl[i].HGCodeCommand.G = -1;
		HGCodeControl.HGCodeDataControl[i].HGCodeCommand.H = -1;

		HGCodeControl.HGCodeDataControl[i].HGCodeParameter.A = 0;
		HGCodeControl.HGCodeDataControl[i].HGCodeParameter.B = 0;
		HGCodeControl.HGCodeDataControl[i].HGCodeParameter.C = 0;
		HGCodeControl.HGCodeDataControl[i].HGCodeParameter.P = 0;
		HGCodeControl.HGCodeDataControl[i].HGCodeParameter.M = 0;
		HGCodeControl.HGCodeDataControl[i].HGCodeParameter.checkSum = 0;
	}*/
	memset(HGCodeControl.HGCodeDataControl,0x00,sizeof(HGCodeDataControl_t)*MAX_COMMAND);
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

	uint16_t rear = HGCodeControl.HGCodeBufferControl.rear;
	uint16_t front = MAX_HGCODE_BUFFER - __HAL_DMA_GET_COUNTER(HGCodeControl.HGCodeDmaHandle);
	uint16_t receiveDataSize;
	int index = rear;
	int data;
	int8_t sign = 1;
	uint8_t buff[20] = {0};
	int j = 0;

	HAL_UART_Transmit(&huart1,(uint8_t*)"hello\r\n",7,1000);

	if( rear < front){
		receiveDataSize = front - rear;
	}else if(rear > front){
		receiveDataSize = front + (MAX_HGCODE_BUFFER - rear);
	}else{
		return;
	}

	//(index+1) % 배열의 사이즈
	for(int i = receiveDataSize; i > 0; i--,index = (index + 1) % MAX_HGCODE_BUFFER){

		//HAL_UART_Transmit_DMA(HGCodeControl.HGCodeUartHandle,&HGCodeBuffer[index],1);

		if(HGCodeBuffer[index] == ';'){
			HGCodeControl.commandCount++;
			HGCodeControl.dataFront = (HGCodeControl.dataFront + 1) % MAX_COMMAND;

			index = (index + 1) % MAX_HGCODE_BUFFER;
			break;
		}

		switch(HGCodeBuffer[index]){
			case 'G':
			case 'H':
			case 'A':
			case 'B':
			case 'C':
			case 'P':
			case 'M':
			case '*':
				data = index;
				index = (index + 1) % MAX_HGCODE_BUFFER;
				if(HGCodeBuffer[index]=='-'){
					sign = -1;
					i--;
					index = (index + 1) % MAX_HGCODE_BUFFER;
				}
				while(HGCodeBuffer[index] != ' '){
					/*if(HGCodeBuffer[index] == ';'){
						HGCodeControl.commandCount++;
						HGCodeControl.dataFront = (HGCodeControl.dataFront + 1) % MAX_COMMAND;
						index = (index + 1) % MAX_HGCODE_BUFFER;

						i = (-1);

						break;
					}*/
					buff[j] = HGCodeBuffer[index];
					j++;
					index = (index + 1) % MAX_HGCODE_BUFFER;
					i--;
				}
				//i--;
				//index = (index + 1) % MAX_HGCODE_BUFFER;
				switch(HGCodeBuffer[data]){
				case 'G':
					HGCodePutData(HGCodeCharToInt(buff,j) * sign,HGCODE_G);
					break;
				case 'H':
					HGCodePutData(HGCodeCharToInt(buff,j) * sign,HGCODE_H);
					break;
				case 'A':
					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_A);
					break;
				case 'B':
					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_B);
					break;
				case 'C':
					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_C);
					break;
				case 'P':
					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_P);
					break;
				case 'M':
					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_M);
					break;
				case '*':
					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_CHECKSUM);
					break;
				}
				j=0;
				sign = 1;
				memset(buff,0x00,sizeof(buff));
				break;
			default:
				break;
		}
		//index = (index + 1) % MAX_HGCODE_BUFFER;
		//i--;
	}
	HGCodeControl.HGCodeBufferControl.rear = index;
}

uint16_t HGCodeGetCommandCount(void){
	return HGCodeControl.commandCount;
}

HGCodeDataControl_t* HGCodeGetCommandData(void){

	uint8_t rear = HGCodeControl.dataRear;

	if(rear == HGCodeControl.dataFront)
		return 0;
	else if(HGCodeControl.commandCount == 0)
		return 0;
	HGCodeControl.dataRear = (HGCodeControl.dataRear + 1) % MAX_COMMAND;

	HGCodeControl.commandCount -= 1;

	return &HGCodeControl.HGCodeDataControl[rear];
}

void HGCodePutData(double data, uint8_t flag){

	switch(flag){
	case HGCODE_G:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeCommand.G = (int16_t)data;
		break;
	case HGCODE_H:
		HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].HGCodeCommand.H = (int16_t)data;
		break;
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

int16_t HGCodeCharToInt(uint8_t* buff, int size){
	int16_t value = 0;
	int a = 1;
	for(int i = size - 1;i >= 0;i--){
		value += (buff[i] - 0x30) * a;
		a*=10;
	}
	return value;
}

double HGCodeCharToDouble(uint8_t* buff ,int size){
	double value = 0.0;
	int a = 1;
	double b = 1;
	for(int i = size - 1;i >= 0;i--){
		if(buff[i] == '.'){
			for(int j = 0; j < size - 1 - i ;j++){
				b *= 0.1;
			}
			value = value * b;
			a = 1;
		}else{
			value += (buff[i] - 0x30) * a;
			a*=10;
		}
	}
	return value;
}

void DecodeTimerInterruptHandler(void){
	if(HGCodeCheckCommandBuffer() == 1){
		HGCodeDecodeCommand();
	}
}

















