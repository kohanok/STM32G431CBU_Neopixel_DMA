/*
 * ws2812.h
 *
 *  Created on: 2019. 6. 4.
 *      Author: HanCheol Cho
 */

#ifndef SRC_HW_DRIVER_WS2812_H_
#define SRC_HW_DRIVER_WS2812_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "main.h"
#include <string.h>

bool ws2812Init(void);
void ws2812Begin(uint32_t led_cnt);
void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif


#endif /* SRC_HW_DRIVER_WS2812_H_ */
