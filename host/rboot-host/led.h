/*
 * led.h
 *
 *  Created on: 13 мая 2017 г.
 *      Author: RomaJam
 */

#ifndef LED_H_
#define LED_H_

#include "app.h"
#include "../rexos/userspace/types.h"
#include "config.h"

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_BLINK
} LED_MODE;

typedef enum {
    LED_COLOR_WHITE = 0,
    LED_COLOR_BLUE,
    LED_COLOR_MAX
} LED_COLOR;

typedef struct {
    bool is_on;
    LED_MODE mode;
    uint8_t pin;
} LED_SETUP;

typedef struct {
    LED_SETUP set[LED_COLOR_MAX];
} LEDS;

void led_init(APP* app);
void led_mode(APP* app, LED_COLOR color, LED_MODE mode);


#endif /* LED_H_ */
