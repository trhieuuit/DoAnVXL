/**
 ******************************************************************************
 * @file           : ws2812.c
 * @brief          : Ws2812 library source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 - 2025 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**
 * Notice, a timer with a DMA driven PWM output will need to be configured
 * before this library is initialized.
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"

#include "ws2812.h"
#include "color_values.h"

/*
 * Update next 24 bits in the dma buffer - assume dma_buffer_pointer is pointing
 * to the buffer that is safe to update.  The dma_buffer_pointer and the call to
 * this function is handled by the dma callbacks.
 */
inline void ws2812_update_buffer(ws2812_handleTypeDef *ws2812, uint16_t *dma_buffer_pointer) {

#ifdef BUFF_GPIO_Port
	HAL_GPIO_WritePin(BUFF_GPIO_Port, BUFF_Pin, GPIO_PIN_SET);
#endif


    ++ws2812->dma_cbs;

    if (ws2812->led_state == LED_RES) {


        if (ws2812->zero_halves < 2) {
            memset(dma_buffer_pointer, 0, 2 * BUFFER_SIZE);
            ws2812->zero_halves++;
        }

        ws2812->res_cnt++;

        if (ws2812->res_cnt >= LED_RESET_CYCLES) {
            ws2812->led_cnt = 0;
            if (ws2812->is_dirty) {
                ws2812->is_dirty = false;
                ws2812->led_state = LED_DAT;
            } else {
                ws2812->led_state = LED_IDL;
            }
        }

    } else if (ws2812->led_state == LED_IDL) {

        if (ws2812->is_dirty) {
            ws2812->is_dirty = false;
            ws2812->led_state = LED_DAT;
        }

    } else {

        ++ws2812->dat_cbs;


        uint8_t *led = (uint8_t*) &ws2812->led[3 * ws2812->led_cnt];

        for (uint8_t c = 0; c < 3; c++) {
            memcpy(dma_buffer_pointer, color_value[led[c]], 16);
            dma_buffer_pointer += 8;

        }

        ws2812->led_cnt++;
        if (ws2812->led_cnt >= ws2812->leds) {
            ws2812->led_cnt = 0;
            ws2812->zero_halves = 0;
            ws2812->res_cnt = 0;
            ws2812->led_state = LED_RES;
        }

    }

#ifdef BUFF_GPIO_Port
	HAL_GPIO_WritePin(BUFF_GPIO_Port, BUFF_Pin, GPIO_PIN_RESET);
#endif

}

ws2812_resultTypeDef zeroLedValues(ws2812_handleTypeDef *ws2812) {
    ws2812_resultTypeDef res = WS2812_Ok;
    memset(ws2812->led, 0, ws2812->leds * 3);
    ws2812->is_dirty = true;
    return res;
}

ws2812_resultTypeDef setLedValue(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t col, uint8_t value) {
    ws2812_resultTypeDef res = WS2812_Ok;
    if (led < ws2812->leds) {
        ws2812->led[3 * led + col] = value;
        ws2812->is_dirty = true;
    } else {
        res = WS2812_Err;
    }
    return res;
}

ws2812_resultTypeDef setLedValues(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t r, uint8_t g, uint8_t b) {
    ws2812_resultTypeDef res = WS2812_Ok;
    if (led < ws2812->leds) {
        ws2812->led[3 * led + RL] = r;
        ws2812->led[3 * led + GL] = g;
        ws2812->led[3 * led + BL] = b;
        ws2812->is_dirty = true;
    } else {
        res = WS2812_Err;
    }
    return res;
}

ws2812_resultTypeDef ws2812_init(ws2812_handleTypeDef *ws2812, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t leds) {

    ws2812_resultTypeDef res = WS2812_Ok;

    ws2812->timer = timer;

    ws2812->channel = channel;

    ws2812->leds = leds;

    ws2812->led_state = LED_RES;
    ws2812->is_dirty = 0;
    ws2812->zero_halves = 2;

    ws2812->led = malloc(leds * 3);
    if (ws2812->led != NULL) {

        memset(ws2812->led, 0, leds * 3);

        HAL_TIM_PWM_Start_DMA(timer, channel, (uint32_t*) ws2812->dma_buffer, BUFFER_SIZE * 2);

    } else {
        res = WS2812_Mem;
    }

    return res;

}

/* 
 * vim: ts=4 nowrap
 */
