/*
 * stm32_pin.c
 *
 *  Created on: 8 θώνÿ 2017 γ.
 *      Author: RLeonov
 */


#include "stm32.h"
#include "stm32_pin.h"

typedef GPIO_TypeDef* GPIO_TypeDef_P;

#define GPIO_PORT(pin)                                          (pin / 16)
#define GPIO_PIN(pin)                                           (pin & 15)

#if defined(STM32F1)
const GPIO_TypeDef_P GPIO[GPIO_COUNT] =                         {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG};
static const unsigned int GPIO_POWER_PINS[GPIO_COUNT] =         {2, 3, 4, 5, 6, 7, 8};
#define GPIO_POWER_PORT                                         RCC->APB2ENR
#elif defined(STM32F0)
const GPIO_TypeDef_P GPIO[GPIO_COUNT] =                         {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF};
static const unsigned int GPIO_POWER_PINS[GPIO_COUNT] =         {17, 18, 19, 20, 21, 22};
#define GPIO_POWER_PORT                                         RCC->AHBENR
#elif defined(STM32F2)
const GPIO_TypeDef_P GPIO[GPIO_COUNT] =                         {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};
static const unsigned int GPIO_POWER_PINS[GPIO_COUNT] =         {0, 1, 2, 3, 4, 5, 6, 7, 8};
#define GPIO_POWER_PORT                                         RCC->AHB1ENR
#elif defined(STM32F4)
const GPIO_TypeDef_P GPIO[GPIO_COUNT] =                         {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK};
static const unsigned int GPIO_POWER_PINS[GPIO_COUNT] =         {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
#define GPIO_POWER_PORT                                         RCC->AHB1ENR
#elif defined(STM32L0)
const GPIO_TypeDef_P GPIO[8] =                                  {GPIOA, GPIOB, GPIOC, GPIOD, 0, 0, 0, GPIOH};
#elif defined(STM32L1)
const GPIO_TypeDef_P GPIO[4] =                                  {GPIOA, GPIOB, GPIOC, GPIOD};
static const unsigned int GPIO_POWER_PINS[GPIO_COUNT] =         {0, 1, 2, 3};
#define GPIO_POWER_PORT                                         RCC->AHBENR
#endif

#if defined(STM32F1)
#define GPIO_CR(pin)                                            (*((unsigned int*)((unsigned int)(GPIO[GPIO_PORT(pin)]) + 4 * ((GPIO_PIN(pin)) / 8))))
#define GPIO_CR_SET(pin, mode)                                  GPIO_CR(pin) &= ~(0xful << ((GPIO_PIN(pin) % 8) * 4ul)); \
                                                                GPIO_CR(pin) |= ((unsigned int)mode << ((GPIO_PIN(pin) % 8) * 4ul))

#define GPIO_ODR_SET(pin, mode)                                 GPIO[GPIO_PORT(pin)]->ODR &= ~(1 << GPIO_PIN(pin)); \
                                                                GPIO[GPIO_PORT(pin)]->ODR |= mode << GPIO_PIN(pin)

void pin_enable(PIN pin, STM32_GPIO_MODE mode, bool pullup)
{
    GPIO_POWER_PORT |= 1 << GPIO_POWER_PINS[GPIO_PORT(pin)];
    GPIO_CR_SET(pin, mode);
    GPIO_ODR_SET(pin, pullup);
}


#else
#define GPIO_SET_MODE(pin, mode)                                GPIO[GPIO_PORT(pin)]->MODER &= ~(3 << (GPIO_PIN(pin) * 2)); \
                                                                GPIO[GPIO_PORT(pin)]->MODER |= ((mode) << (GPIO_PIN(pin) * 2))

#define GPIO_SET_OT(pin, mode)                                  GPIO[GPIO_PORT(pin)]->OTYPER &= ~(1 << GPIO_PIN(pin)); \
                                                                GPIO[GPIO_PORT(pin)]->OTYPER |= ((mode) << GPIO_PIN(pin))

#define GPIO_SET_SPEED(pin, mode)                               GPIO[GPIO_PORT(pin)]->OSPEEDR &= ~(3 << (GPIO_PIN(pin) * 2)); \
                                                                GPIO[GPIO_PORT(pin)]->OSPEEDR |= ((mode) << (GPIO_PIN(pin) * 2))

#define GPIO_SET_PUPD(pin, mode)                                GPIO[GPIO_PORT(pin)]->PUPDR &= ~(3 << (GPIO_PIN(pin) * 2)); \
                                                                GPIO[GPIO_PORT(pin)]->PUPDR |= ((mode) << (GPIO_PIN(pin) * 2))

#define GPIO_AFR(pin)                                           (*((unsigned int*)((unsigned int)(GPIO[GPIO_PORT(pin)]) + 0x20 + 4 * ((GPIO_PIN(pin)) / 8))))
#define GPIO_AFR_SET(pin, mode)                                 GPIO_AFR(pin) &= ~(0xful << ((GPIO_PIN(pin) % 8) * 4ul)); \
                                                                GPIO_AFR(pin) |= ((unsigned int)(mode) << ((GPIO_PIN(pin) % 8) * 4ul))

void pin_enable(PIN pin, unsigned int mode, AF af)
{
#if defined(STM32L0)
    RCC->IOPENR |= 1 << GPIO_PORT(pin);
#else
    GPIO_POWER_PORT |= 1 << GPIO_POWER_PINS[GPIO_PORT(pin)];
#endif
    GPIO_SET_MODE(pin, (mode >> 0) & 3);
    GPIO_SET_OT(pin, (mode >> 2) & 1);
    GPIO_SET_SPEED(pin, (mode >> 3) & 3);
    GPIO_SET_PUPD(pin, (mode >> 5) & 3);
    GPIO_AFR_SET(pin, af);
}
#endif

void gpio_enable(unsigned int pin, GPIO_MODE mode)
{

#if defined(STM32F1)
    switch (mode)
    {
    case GPIO_MODE_OUT:
        pin_enable(pin, STM32_GPIO_MODE_OUTPUT_PUSH_PULL_50MHZ, false);
        break;
    case GPIO_MODE_IN_FLOAT:
        pin_enable(pin, STM32_GPIO_MODE_INPUT_FLOAT, false);
        break;
    case GPIO_MODE_IN_PULLUP:
        pin_enable(pin, STM32_GPIO_MODE_INPUT_PULL, true);
        break;
    case GPIO_MODE_IN_PULLDOWN:
        pin_enable(pin, STM32_GPIO_MODE_INPUT_PULL, false);
        break;
    }
#else
    switch (mode)
    {
    case GPIO_MODE_OUT:
        pin_enable(pin, STM32_GPIO_MODE_OUTPUT | GPIO_OT_PUSH_PULL | GPIO_SPEED_HIGH | GPIO_PUPD_NO_PULLUP, AF0);
        break;
    case GPIO_MODE_IN_FLOAT:
        pin_enable(pin, STM32_GPIO_MODE_INPUT | GPIO_SPEED_HIGH | GPIO_PUPD_NO_PULLUP, AF0);
        break;
    case GPIO_MODE_IN_PULLUP:
        pin_enable(pin, STM32_GPIO_MODE_INPUT | GPIO_SPEED_HIGH | GPIO_PUPD_PULLUP, AF0);
        break;
    case GPIO_MODE_IN_PULLDOWN:
        pin_enable(pin, STM32_GPIO_MODE_INPUT | GPIO_SPEED_HIGH | GPIO_PUPD_PULLDOWN, AF0);
        break;
    }
#endif
}


void pin_set(PIN pin)
{
#if defined(STM32F1) || defined(STM32L1) || defined (STM32L0) || defined(STM32F0)
    GPIO[GPIO_PORT(pin)]->BSRR = 1 << GPIO_PIN(pin);
#else
    GPIO[GPIO_PORT(pin)]->BSRRL = 1 << GPIO_PIN(pin);
#endif
}

void pin_reset(PIN pin)
{
#if defined(STM32F1) || defined(STM32L1) || defined (STM32L0) || defined(STM32F0)
    GPIO[GPIO_PORT(pin)]->BSRR = 1 << (GPIO_PIN(pin) + 16);
#else
    GPIO[GPIO_PORT(pin)]->BSRRH = 1 << GPIO_PIN(pin);
#endif
}

bool pin_get(PIN pin)
{
    return (GPIO[GPIO_PORT(pin)]->IDR >> GPIO_PIN(pin)) & 1;
}
