/**
 ******************************************************************************
 * @file           : ws2812_demos.h
 * @brief          : Ws2812b demos source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lbthomsen@gmail.com>.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"

#include "ws2812.h"
#include "ws2812_demos.h"

uint8_t active_demo = 0;

const uint8_t led_line_colors[][3] = {
        { 10, 0, 0 },
        { 0, 10, 0 },
        { 0, 0, 10 },
        { 10, 10, 0 },
        { 0, 10, 10 },
        { 10, 0, 10 },
        { 10, 10, 10 }
};

void advanced_ripple_effect(ws2812_handleTypeDef *ws, float radius, uint8_t color1[3], uint8_t color2[3], float pulse_width) {
    const float center_x = 3.5f;
    const float center_y = 3.5f;

    for (uint8_t x = 0; x < 8; x++) {
        for (uint8_t y = 0; y < 8; y++) {
            float dx = x - center_x;
            float dy = y - center_y;
            float distance = sqrtf(dx*dx + dy*dy);

            // Tính độ sáng với vòng lan tỏa dày hơn
            float brightness = 0;
            if (distance <= radius && distance > radius - pulse_width) {
                brightness = 1.0f - (radius - distance)/pulse_width;

                // Thêm hiệu ứng mờ dần về phía rìa
                brightness *= (1.0f - (radius - distance)/pulse_width);
            }

            // Tính gradient màu
            float gradient_pos = (radius - distance)/pulse_width;
            uint8_t r = color1[0] * (1.0f - gradient_pos) + color2[0] * gradient_pos;
            uint8_t g = color1[1] * (1.0f - gradient_pos) + color2[1] * gradient_pos;
            uint8_t b = color1[2] * (1.0f - gradient_pos) + color2[2] * gradient_pos;

            // Áp dụng độ sáng
            r *= brightness;
            g *= brightness;
            b *= brightness;

            // Tính chỉ số LED (kiểu snake)
            uint8_t index = x * 8 + ((x % 2) ? (7 - y) : y);
            setLedValues(ws, index, r, g, b);
        }
    }
    ws->is_dirty = 1;
}




void smiley_face_effect(ws2812_handleTypeDef *ws, uint8_t color[3]) {
    // Máº«u máº·t cÆ°á»i (1 = báº­t LED, 0 = táº¯t LED)
    const uint8_t smiley_pattern[8][8] = {
    		{0, 0, 0, 0, 0, 0, 0, 0},
    		{0, 1, 1, 0, 0, 1, 1, 0},
    		{1, 1, 1, 1, 1, 1, 1, 1},
    		{1, 1, 1, 1, 1, 1, 1, 1},
    		{1, 1, 1, 1, 1, 1, 1, 1},
    		{0, 1, 1, 1, 1, 1, 1, 0},
    		{0, 0, 1, 1, 1, 1, 0, 0},
    		{0, 0, 0, 1, 1, 0, 0, 0}
    };

    for (uint8_t x = 0; x < 8; x++) {
        for (uint8_t y = 0; y < 8; y++) {
            uint8_t r = 0, g = 0, b = 0;

            if (smiley_pattern[x][y]) {
                r = color[0];
                g = color[1];
                b = color[2];
            }

            uint8_t index = x * 8 + ((x % 2) ? (7 - y) : y);
            setLedValues(ws, index, r, g, b);
        }
    }
    ws->is_dirty = 1;
}

void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
    float c = v * s;
    float x = c * (1 - fabs(fmodf(h * 6, 2) - 1));
    float m = v - c;

    float r_, g_, b_;

    if (h < 1.0f/6) {
        r_ = c; g_ = x; b_ = 0;
    } else if (h < 2.0f/6) {
        r_ = x; g_ = c; b_ = 0;
    } else if (h < 3.0f/6) {
        r_ = 0; g_ = c; b_ = x;
    } else if (h < 4.0f/6) {
        r_ = 0; g_ = x; b_ = c;
    } else if (h < 5.0f/6) {
        r_ = x; g_ = 0; b_ = c;
    } else {
        r_ = c; g_ = 0; b_ = x;
    }

    *r = (uint8_t)((r_ + m) * 255);
    *g = (uint8_t)((g_ + m) * 255);
    *b = (uint8_t)((b_ + m) * 255);
}


void ws2812_demos_set(ws2812_handleTypeDef *ws2812, uint8_t demo ) {
    active_demo = demo;
}

void ws2812_demos_tick(ws2812_handleTypeDef *ws2812,  uint8_t *heights) {

    static const uint32_t led_interval = 20;

    static uint16_t line_led = 0;
    static uint32_t line_count = 0;
    static uint8_t line_color = 0;
    static uint32_t next_led = led_interval;

    static float hue_shift = 0;
       static uint8_t internal_heights[8] = {0};
       static uint8_t direction[8] = {1,1,1,1,1,1,1,1};


    static float radius = 0;
    	    static uint32_t last_tick = 0;
    	    static uint8_t color_pair = 0;
    	    static float hue = 0;

    uint32_t now = uwTick;
    switch (active_demo) {
    case WS2812_DEMO_LINE: {
        static uint32_t next_led = 0;
        static uint16_t line_led = 0;
        static uint8_t line_color = 0;

        // Tính tốc độ từ dữ liệu heights
        uint8_t speed_factor = 4;  // giá trị mặc định nếu không có dữ liệu
        if (heights != NULL) {
            uint8_t max_height = 0;
            for (int i = 0; i < 8; i++) {
                if (heights[i] > max_height) max_height = heights[i];
            }
            // Tăng nhanh khi âm lượng/tần số cao
            speed_factor = 8 - max_height;
            if (speed_factor < 1) speed_factor = 1;
        }

        uint32_t dynamic_interval = speed_factor * 1;  // đơn vị ms

        if (now >= next_led) {
            uint8_t index = line_led;
            setLedValues(ws2812, index, led_line_colors[line_color][0], led_line_colors[line_color][1], led_line_colors[line_color][2]);
            ws2812->is_dirty = 1;

            line_led++;
            if (line_led >= LEDS) {
                line_led = 0;
                line_color++;
                if (line_color >= sizeof(led_line_colors) / sizeof(led_line_colors[0]))
                    line_color = 0;
            }

            next_led = now + dynamic_interval;
        }
        break;
    }

    case WS2812_DEMO_ADV_RIPPLE: {

        const float pulse_width = 3.5f;

        static float radius = 0.0f;
        static float hue = 0.0f;
        static uint8_t color_pair = 0;

        int Max = 0;
        for (int i = 0; i < 8; i++) {
            if (heights[i] > Max) Max = heights[i];
        }

        // Tăng tốc ripple theo âm thanh
//        float radius_step = 0.20f + 0.1f * Max;
        float radius_step = 0.20f + 0.08f * Max + 0.2f * (Max >= 6 ? (Max - 5) : 0);
        radius += radius_step;

        // Vòng lặp lại
        if (radius > 6.0f) {
            radius = 0;
            color_pair = (color_pair + 1) % 3;
        }

        hue += 0.01f;

        uint8_t color1[3], color2[3];
        hsv_to_rgb(fmodf(hue, 1.0f), 1.0f, 1.0f, &color1[0], &color1[1], &color1[2]);
        hsv_to_rgb(fmodf(hue + 0.3f, 1.0f), 1.0f, 1.0f, &color2[0], &color2[1], &color2[2]);

        advanced_ripple_effect(ws2812, radius, color1, color2, pulse_width);
        break;
    }

    case WS2812_WAVE_BASIC:


    	    static float hue_shift = 0.0f;

    	    if (now - last_tick >= 100) {  // điều chỉnh tốc độ đổi màu tại đây

    	        hue_shift += 0.01f;
    	        if (hue_shift > 1.0f) hue_shift = 0;

    	        for (uint8_t x = 0; x < 8; x++) {
    	            uint8_t h = heights[x];
    	            for (uint8_t y = 0; y < 8; y++) {
    	                uint8_t r = 0, g = 0, b = 0;

    	                if (y < h) {
    	                    float blend = (float)y / 8.0f;

    	                    // Tính hue theo thời gian + chiều cao
    	                    float hue = fmodf(hue_shift + (blend / 2.0f), 1.0f);
    	                    float saturation = 1.0f;
    	                    float value = 0.2f + 0.8f * blend;

    	                    hsv_to_rgb(hue, saturation, value, &r, &g, &b);
    	                }

    	                // Tính index kiểu zigzag hoặc column-major
    	                uint8_t index = x * 8 + y;
    	                setLedValues(ws2812, index, r, g, b);
    	            }
    	        }

    	        ws2812->is_dirty = 1;
    	        last_tick = now;
    	    }
    	    break;

    case WS2812_WAVE_MIRROR: {

        const uint8_t max_height = 8;

        static float hue = 0.0f;

        hue += 0.01f;
        if (hue > 1.0f) hue = 0;

        for (uint8_t x = 0; x < 4; x++) {
            uint8_t h = heights[x];
            if (h > max_height) h = max_height;

            float hue_offset = fmodf(hue + x * 0.05f, 1.0f);

            for (uint8_t y = 0; y < 8; y++) {
                uint8_t r = 0, g = 0, b = 0;

                if (y < h) {
                    float value = 0.3f + 0.7f * (float)y / 7.0f;
                    hsv_to_rgb(hue_offset, 1.0f, value, &r, &g, &b);
                }

                // Bên trái
                uint8_t index_left = x * 8 + ((x % 2) ? (7 - y) : y);
                setLedValues(ws2812, index_left, r, g, b);

                // Bên phải đối xứng
                uint8_t mirror_x = 7 - x;
                uint8_t index_right = mirror_x * 8 + ((mirror_x % 2) ? (7 - y) : y);
                setLedValues(ws2812, index_right, r, g, b);
            }
        }

        ws2812->is_dirty = 1;
        break;
    }
    case WS2812_MATRIX_CENTER_MIRROR: {

        static float hue_shift = 0.0f;
        hue_shift += 0.01f;
        if (hue_shift > 1.0f) hue_shift = 0.0f;

        uint8_t r, g, b;
        hsv_to_rgb(hue_shift, 1.0f, 1.0f, &r, &g, &b);

        uint8_t center_row_top = 3;  // hàng 3 (tính từ 0)
        uint8_t center_row_bottom = 4; // hàng 4 (tính từ 0)

        for (uint8_t d = 0; d < 4; d++) {
            uint8_t h = heights[d];

            // Phần đối xứng bên trên từ hàng center_row_top
            int8_t row_up = center_row_top - d;
            if (row_up >= 0) {
                for (uint8_t x = 0; x < 8; x++) {
                    uint8_t val_r = (x < h) ? r : 0;
                    uint8_t val_g = (x < h) ? g : 0;
                    uint8_t val_b = (x < h) ? b : 0;
                    uint8_t index = (x % 2 == 0) ? (x * 8 + row_up) : (x * 8 + (7 - row_up));
                    setLedValues(ws2812, index, val_r, val_g, val_b);
                }
            }

            // Phần đối xứng bên dưới từ hàng center_row_bottom
            int8_t row_down = center_row_bottom + d;
            if (row_down <= 7) {
                for (uint8_t x = 0; x < 8; x++) {
                    uint8_t val_r = (x < h) ? r : 0;
                    uint8_t val_g = (x < h) ? g : 0;
                    uint8_t val_b = (x < h) ? b : 0;
                    uint8_t index = (x % 2 == 0) ? (x * 8 + row_down) : (x * 8 + (7 - row_down));
                    setLedValues(ws2812, index, val_r, val_g, val_b);
                }
            }
        }

        ws2812->is_dirty = 1;
        break;
    }

    case WS2812_SMILEY:
    	 static uint32_t last_tick = 0;
    	    static float hue = 0;

    	    // Tá»‘c Ä‘á»™ thay Ä‘á»•i mÃ u (ms)
    	    if (HAL_GetTick() - last_tick >= 100) {
    	        // Chuyá»ƒn mÃ u liÃªn tá»¥c
    	        hue += 0.01f;
    	        if (hue > 1.0f) hue = 0;

    	        // Chuyá»ƒn HSV sang RGB
    	        uint8_t color[3];
    	        hsv_to_rgb(hue, 1.0f, 1.0f, &color[0], &color[1], &color[2]);

    	        // Hiá»ƒn thá»‹ máº·t cÆ°á»i
    	        smiley_face_effect(ws2812, color);

    	        last_tick = HAL_GetTick();
    	    }
    	    break;
    default:
        // De nothing really

    }
}
