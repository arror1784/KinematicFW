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

	memset(HGCodeControl.HGCodeDataControl,0x00,sizeof(commandFormat_t)*MAX_COMMAND);
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
	uint8_t checksum = 0;
	int transmitErrorFlag = 0;
	int8_t sign = 1;
	int j = 0;
	char buff[50] = {0};
	commandFormat_t temp = {0};
	uint8_t response[9];
	response[0] = 0x02;
	response[1] = 'a';
	response[2] = 'r';
	response[3] = 'r';
	response[4] = 'o';
	response[5] = 'r';
	response[6] = 0x26;
	response[7] = 0x03;

	if( rear < front){
		receiveDataSize = front - rear;
	}else if(rear > front){
		receiveDataSize = front + (MAX_HGCODE_BUFFER - rear);
	}else{
		return;
	}
//	HGCodeControl.HGCodeDataControl[HGCodeControl.dataFront].
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
					HAL_UART_Transmit_IT(&huart1,"transmit error\r\n",16);
					HAL_UART_Transmit_IT(&huart2,response,8);
//					index = (index + 25) % MAX_HGCODE_BUFFER;
					break;
				}
				temp.G = HGCodeBuffer[(index + 1) % MAX_HGCODE_BUFFER];
				temp.H = HGCodeBuffer[(index + 2) % MAX_HGCODE_BUFFER];

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

				HGCodePutData(temp.G,HGCODE_G);
				HGCodePutData(temp.H,HGCODE_H);
				HGCodePutData((double)temp.A.db,HGCODE_A);
				HGCodePutData((double)temp.B.db,HGCODE_B);
				HGCodePutData((double)temp.C.db,HGCODE_C);
				HGCodePutData((double)temp.M.db,HGCODE_M);
				HGCodePutData((double)temp.P.db,HGCODE_P);

				HGCodeControl.dataFront = (HGCodeControl.dataFront + 1) % MAX_COMMAND;
				HGCodeControl.commandCount++;
				break;
			}
		}
		HGCodeControl.HGCodeBufferControl.rear = (index + 25) % MAX_HGCODE_BUFFER;
	}
	/*
	//(index+1) % 배열의 사이즈
	for(int i = receiveDataSize; i > 0; i--,index = (index + 1) % MAX_HGCODE_BUFFER){

		if(HGCodeBuffer[index] == ';'){
			HGCodeControl.commandCount++;
			HGCodeControl.dataFront = (HGCodeControl.dataFront + 1) % MAX_COMMAND;

			index = (index + 1) % MAX_HGCODE_BUFFER;
			break;
		}
		switch(HGCodeBuffer[index]){
			case 'G':
			case 'H':
				checksum = 0;
			case 'A':
			case 'B':
			case 'C':
			case 'P':
			case 'M':
				checksum += HGCodeBuffer[index];
				data = index;
				index = (index + 1) % MAX_HGCODE_BUFFER;
				if(HGCodeBuffer[index]=='-'){
					sign = -1;
					i--;
					checksum += HGCodeBuffer[index];
					index = (index + 1) % MAX_HGCODE_BUFFER;

				}
				while(HGCodeBuffer[index] != ' ' && HGCodeBuffer[index] != ';' ){
					buff[j] = HGCodeBuffer[index];
					j++;
					checksum += HGCodeBuffer[index];
					index = (index + 1) % MAX_HGCODE_BUFFER;
					i--;
				}
				checksum += HGCodeBuffer[index];
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
//					HGCodePutData(HGCodeCharToDouble(buff,j) * sign,HGCODE_CHECKSUM);
					break;
				}
				j=0;
				sign = 1;
				break;
			case '*':
				index = (index + 1) % MAX_HGCODE_BUFFER;
				if(checksum == HGCodeBuffer[index]){
//					HAL_UART_Transmit(&huart1,"transmit sucess\r\n",17,1000);
					index = (index + 1) % MAX_HGCODE_BUFFER;
				}else{
					HAL_UART_Transmit_IT(&huart1,"transmit error\r\n",16);
					HAL_UART_Transmit_IT(&huart2,"transmit error",16);
					while(HGCodeBuffer[index] != ';'){
						index = (index + 1) % MAX_HGCODE_BUFFER;
					}
					transmitErrorFlag = 1;
				}
				break;
			default:
				break;
		}
		if(transmitErrorFlag == 1){
			transmitErrorFlag = 0;
			index = (index + 1) % MAX_HGCODE_BUFFER;
			break;
		}else if(HGCodeBuffer[index] == ';'){
			HGCodeControl.commandCount++;
			HGCodeControl.dataFront = (HGCodeControl.dataFront + 1) % MAX_COMMAND;
			index = (index + 1) % MAX_HGCODE_BUFFER;
			break;
		}
	}
	HGCodeControl.HGCodeBufferControl.rear = index;
	*/
}

uint16_t HGCodeGetCommandCount (void){
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
//
//int16_t HGCodeCharToInt(uint8_t* buff, int size){
//	int16_t value = 0;
//	int a = 1;
//	for(int i = size - 1;i >= 0;i--){
//		value += (buff[i] - 0x30) * a;
//		a*=10;
//	}
//	return value;
//}
//
//double HGCodeCharToDouble(uint8_t* buff ,int size){
//	double value = 0.0;
//	int a = 1;
//	double b = 1;
//	for(int i = size - 1;i >= 0;i--){
//		if(buff[i] == '.'){
//			for(int j = 0; j < size - 1 - i ;j++){
//				b *= 0.1;
//			}
//			value = value * b;
//			a = 1;
//		}else{
//			value += (buff[i] - 0x30) * a;
//			a*=10;
//		}
//	}
//	return value;
//}

void DecodeTimerInterruptHandler(void){
	if(HGCodeCheckCommandBuffer() == 1){
		HGCodeDecodeCommand();
	}
}

