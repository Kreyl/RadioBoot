/*
 * led.c
 *
 *  Created on: 13 мая 2017 г.
 *      Author: RomaJam
 */

#include "../rexos/userspace/stm32/stm32_driver.h"
#include "../rexos/userspace/gpio.h"
#include "../rexos/userspace/stdio.h"
#include "../rexos/userspace/htimer.h"
#include "../rexos/userspace/irq.h"
#include "app_private.h"
#include "led.h"


static inline void led_toggle(LEDS* leds, LED_COLOR color)
{
    if(leds->set[color].is_on)
        gpio_reset_pin(leds->set[color].pin);
    else
        gpio_set_pin(leds->set[color].pin);
    leds->set[color].is_on = !leds->set[color].is_on;

    if(leds->set[color].need_blink > 0)
        leds->set[color].need_blink--;
}


static inline void led_irq(int vector, void* param)
{
    LEDS* leds = (LEDS*)param;
    for(uint8_t i = 0; i < LED_COLOR_MAX; i++)
    {
        if(leds->set[i].mode == LED_MODE_BLINK)
        {
            if(leds->set[i].need_blink > 0)
                led_toggle(leds, i);
            else
            {
                leds->set[i].mode = LED_MODE_ON;
                // disable htimer
                LED_BLINK_TIM_REG->CR1 &= ~TIM_CR1_CEN;
                LED_BLINK_TIM_REG->SR &= ~TIM_SR_UIF;
            }
        }
    }
    LED_BLINK_TIM_REG->SR &= ~TIM_SR_UIF;
}

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

//    pin_enable(app->leds.set[LED_COLOR_WHITE].pin, STM32_GPIO_MODE_AF, AF2);
//    pin_enable(app->leds.set[LED_COLOR_BLUE].pin, STM32_GPIO_MODE_AF, AF2);

    htimer_open(LED_BLINK_TIM, TIMER_IRQ_ENABLE);
    htimer_setup_channel(LED_BLINK_TIM, LED_BLINK_TIM_CHANNEL, TIMER_CHANNEL_GENERAL, 0);

    irq_register(LED_BLINK_IRQ_VECTOR, led_irq, (void*)&app->leds);

#if (APP_LED_DEBUG)
    printf("LED: init\n");
#endif // APP_LED_DEBUG
}

void led_mode(APP* app, LED_COLOR color, LED_MODE mode)
{
#if (APP_LED_DEBUG)
    printf("LED: mode %u\n", mode);
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
            app->leds.set[color].need_blink = LED_BLINK_TOGGLE_TIMES;
            led_toggle(&app->leds, color);
            htimer_start(LED_BLINK_TIM, TIMER_VALUE_HZ, LED_BLINK_FREQ_HZ);
            break;
    }

    app->leds.set[color].mode = mode;
}
