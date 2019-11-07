/*
 * noePixel.h
 *
 *  Created on: 2019. 5. 7.
 *      Author: user
 */

#ifndef NEOPIXEL_H_
#define NEOPIXEL_H_

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <tim.h>

typedef struct{

	TIM_HandleTypeDef* timerControl;
	DMA_HandleTypeDef* timerDMAControl;
	uint32_t *bitBuff;
	uint16_t ledCount;
	uint32_t channel;

}WS2812BControl_t, *WS2812BControl_p;

extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com;
extern WS2812BControl_t neoPixel;
extern WS2812BControl_p neoPixel_P;

void setNeoPixel(WS2812BControl_t *WS2812BControl,TIM_HandleTypeDef* timerControl,uint32_t channel,DMA_HandleTypeDef* timerDMAControl,uint16_t ledCount);

uint32_t converColorTo32(uint8_t green,uint8_t red,uint8_t blue);
void setColor(WS2812BControl_t *WS2812BControl,uint32_t color, uint16_t ledNo);
void setColorArr(uint32_t* buf, uint32_t color,uint16_t ledNo);

void updateColorBlack(WS2812BControl_t *WS2812BControl);

void updateColor(WS2812BControl_t *WS2812BControl,int8_t mode);
void updateColorBuff(WS2812BControl_t *WS2812BControl,uint32_t *buff,uint32_t length);
void neoPixelPWMPulseInterruptHandler(WS2812BControl_t *WS2812BControl,TIM_HandleTypeDef *htim);
void freeNeoPixel(WS2812BControl_t *WS2812BControl);
uint32_t getNDTR(WS2812BControl_t *WS2812BControl);

void setMode(WS2812BControl_t *WS2812BControl, uint8_t mode/*, uint32_t* buf,uint32_t length*/);
void setModeClear(WS2812BControl_t *WS2812BControl, uint8_t mode);

#endif /* NEOPIXEL_H_ */
