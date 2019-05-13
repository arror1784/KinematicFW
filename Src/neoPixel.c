/*
 * neoPixel.c
 *
 *  Created on: 2019. 5. 7.
 *      Author: user
 */
#include "neoPixel.h"

void setNeoPixel(WS2812BControl_t *WS2812BControl,TIM_HandleTypeDef* timerControl,uint32_t channel,DMA_HandleTypeDef* timerDMAControl,uint16_t ledCount){
	WS2812BControl->bitBuff = (uint32_t*)malloc(sizeof(uint32_t) * 24 * (ledCount + 2));
	memset(WS2812BControl->bitBuff,0x00,sizeof(uint32_t) * 24 * (ledCount + 2));
//	WS2812BControl->bitBuff = bitBuff;
	WS2812BControl->timerControl = timerControl;
	WS2812BControl->channel = channel;
	WS2812BControl->timerDMAControl = timerDMAControl;
	WS2812BControl->ledCount = ledCount;
}

uint32_t converColorTo32(uint8_t green,uint8_t red,uint8_t blue){
	return ((uint32_t)green << 16) | ((uint32_t)red << 8) | ((uint32_t)blue);
}
void setColor(WS2812BControl_t *WS2812BControl, uint32_t color, uint16_t ledNo){
	for(int i=0; i<24; i++){
		if(color&(1<<(23-i))) WS2812BControl->bitBuff[ledNo*24 + i + 24] = 16;
	    else                  WS2812BControl->bitBuff[ledNo*24 + i + 24] = 8 ;
	}
}
void setColorBlack(WS2812BControl_t *WS2812BControl){
	for(int i = 0 ; i < WS2812BControl->ledCount; i++){
		setColor(WS2812BControl,converColorTo32(0,0,0),i);
	}
	updateColor(WS2812BControl);
}
void updateColor(WS2812BControl_t *WS2812BControl){

	HAL_TIM_PWM_Stop_DMA(WS2812BControl->timerControl,WS2812BControl->channel);
//	HAL_TIM_PWM_Start_DMA(WS2812BControl->timerControl,WS2812BControl->channel,WS2812BControl->bitBuff,WS2812BControl->ledCount * 24 + 24);
	HAL_TIM_PWM_Start_DMA(WS2812BControl->timerControl,TIM_CHANNEL_4,WS2812BControl->bitBuff,(WS2812BControl->ledCount * 24) + 48);
	while(getNDTR(WS2812BControl) > 0);
	HAL_TIM_PWM_Stop_DMA(WS2812BControl->timerControl,WS2812BControl->channel);

}
uint32_t getNDTR(WS2812BControl_t *WS2812BControl){
	return WS2812BControl->timerDMAControl->Instance->NDTR;
}
void freeNeoPixel(WS2812BControl_t *WS2812BControl){
	free(WS2812BControl->bitBuff);
}
