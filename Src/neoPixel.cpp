/*
 * neoPixel.c
 *
 *  Created on: 2019. 5. 7.
 *      Author: user
 */
#include "neoPixel.h"

uint32_t  blankLedbuf[10*24+48] = {0};
WS2812BControl_t neoPixel = {0};
WS2812BControl_p neoPixel_P = &neoPixel;

void setNeoPixel(WS2812BControl_t *WS2812BControl,TIM_HandleTypeDef* timerControl,uint32_t channel,DMA_HandleTypeDef* timerDMAControl,uint16_t ledCount,TIM_HandleTypeDef* blankTimer){
	WS2812BControl->bitBuff = (uint32_t*)malloc(sizeof(uint32_t) * 24 * (ledCount + 2));
	memset(WS2812BControl->bitBuff,0x00,sizeof(uint32_t) * 24 * (ledCount + 2));
	WS2812BControl->timerControl = timerControl;
	WS2812BControl->channel = channel;
	WS2812BControl->timerDMAControl = timerDMAControl;
	WS2812BControl->ledCount = ledCount;
	WS2812BControl->mode=0;
	WS2812BControl->blankCount=0;
	WS2812BControl->blankFlag=0;
	WS2812BControl->blankTimer=blankTimer;
}

uint32_t converColorTo32(uint8_t green,uint8_t red,uint8_t blue){
	return ((uint32_t)green << 16) | ((uint32_t)red << 8) | ((uint32_t)blue);
}
void setColor(WS2812BControl_t *WS2812BControl, uint32_t color, uint16_t ledNo){
	for(int i=0; i<24; i++){
		if(color&(1<<(23-i))) WS2812BControl->bitBuff[ledNo*24 + i + 24] = (uint32_t)76;
	    else                  WS2812BControl->bitBuff[ledNo*24 + i + 24] = (uint32_t)36 ;
	}
}
void setColorArr(uint32_t* buf, uint32_t color,uint16_t ledNo){
	for(int i=0; i<24; i++){
		if(color&(1<<(23-i))) buf[ledNo*24 + i] = (uint32_t)76;
	    else                  buf[ledNo*24 + i] = (uint32_t)36;
	}
}
void updateColorBlack(WS2812BControl_t *WS2812BControl){
	for(int i = 0 ; i < WS2812BControl->ledCount; i++){
		setColor(WS2812BControl,converColorTo32((uint8_t)0x00,(uint8_t)0x00,(uint8_t)0x00),i);
	}
	updateColor(WS2812BControl,0);
}
void updateColor(WS2812BControl_t *WS2812BControl,int8_t mode){
	switch(mode){
	case 0:
		setMode(WS2812BControl,0);
	case -1:
		HAL_TIM_PWM_Stop_DMA(WS2812BControl->timerControl,WS2812BControl->channel);
		HAL_TIM_PWM_Start_DMA(WS2812BControl->timerControl,WS2812BControl->channel,WS2812BControl->bitBuff,(WS2812BControl->ledCount * 24) + 48);
		while(WS2812BControl->timerDMAControl->Instance->NDTR > 0);
		HAL_TIM_PWM_Stop_DMA(WS2812BControl->timerControl,WS2812BControl->channel);
		break;
	case 1:
		setMode(WS2812BControl,1);
	}
}
void updateColorBuff(WS2812BControl_t *WS2812BControl,uint32_t *buff,uint32_t length,int8_t mode){
	switch(mode){
	case 0:
		setMode(WS2812BControl,0);
	case -1:
		HAL_TIM_PWM_Stop_DMA(WS2812BControl->timerControl,WS2812BControl->channel);
		HAL_TIM_PWM_Start_DMA(WS2812BControl->timerControl,WS2812BControl->channel,buff,length);
		while(WS2812BControl->timerDMAControl->Instance->NDTR > 0);
		HAL_TIM_PWM_Stop_DMA(WS2812BControl->timerControl,WS2812BControl->channel);
		break;
	case 1:
		setMode(WS2812BControl,1);
	}
}
uint32_t getNDTR(WS2812BControl_t *WS2812BControl){
	return WS2812BControl->timerDMAControl->Instance->NDTR;
}

void setMode(WS2812BControl_t *WS2812BControl, uint8_t mode){
	switch(mode){
	case 0: // 일반 모드
		WS2812BControl->mode = 0;
		HAL_TIM_Base_Stop_IT(WS2812BControl->blankTimer);
		break;
	case 1: // 파란점멸  모드
		WS2812BControl->mode = 1;
		HAL_TIM_Base_Start_IT(WS2812BControl->blankTimer);
		break;
	default:
		break;
	}
}
void setModeClear(WS2812BControl_t *WS2812BControl, uint8_t mode){

}
void freeNeoPixel(WS2812BControl_t *WS2812BControl){
	free(WS2812BControl->bitBuff);
}

void NPEXITHandle(WS2812BControl_t *WS2812BControl){
	uint32_t buff[10*24+48] = {0};
	if(WS2812BControl->mode == 1){
		if(WS2812BControl->blankCount == 4){
			if(WS2812BControl->blankFlag)
				updateColor(neoPixel_P,-1);
			else{
				for(int i = 0 ; i < neoPixel_P->ledCount; i++){
					setColorArr(&buff[24],converColorTo32(0,0,0),i);
				}
				updateColorBuff(neoPixel_P,buff,10*24+48,-1);
			}
			WS2812BControl->blankFlag = ~WS2812BControl->blankFlag;

			WS2812BControl->blankCount = 0;
		}else{
			WS2812BControl->blankCount++;
		}
	}
}
