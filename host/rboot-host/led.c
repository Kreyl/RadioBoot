/*
 * led.c
 *
 *  Created on: 13 мая 2017 г.
 *      Author: RomaJam
 */

#include "app_private.h"
#include "../rexos/userspace/stm32/stm32_driver.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/userspace/stdio.h"
#include "led.h"


void led_init(APP* app)
{
    app->leds.set[LED_COLOR_WHITE].pin = LED_WHITE_PIN;
    app->leds.set[LED_COLOR_BLUE].pin = LED_BLUE_PIN;

    for(uint8_t i = 0; i < LED_COLOR_MAX; i++)
    {
        app->leds.set[i].is_on = false;
        app->leds.set[i].mode = LED_MODE_OFF;
        gpio_enable_pin(app->leds.set[i].pin, GPIO_MODE_OUT);
        gpio_reset_pin(app->leds.set[i].pin);
    }

#if (APP_LED_DEBUG)
    printf("LED: init\n");
#endif // APP_LED_DEBUG

}

void led_mode(APP* app, LED_COLOR color, LED_MODE mode)
{
#if (APP_LED_DEBUG)
    printf("LED: mode\n");
#endif // APP_LED_DEBUG

    if(color > LED_COLOR_MAX)
        return;

    if(app->leds.set[color].mode == mode)
        return;

    switch(app->leds.set[color].mode)
    {
        case LED_MODE_OFF:
        case LED_MODE_ON:
        case LED_MODE_BLINK:
            break;
    }

    switch(mode)
    {
        case LED_MODE_OFF:
            gpio_reset_pin(app->leds.set[color].pin);
            app->leds.set[color].is_on = false;
            break;
        case LED_MODE_ON:
            gpio_set_pin(app->leds.set[color].pin);
            app->leds.set[color].is_on = true;
            break;
        case LED_MODE_BLINK:
            break;
    }

    app->leds.set[color].mode = mode;
}
