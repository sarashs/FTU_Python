/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define TCC1_out GPIO(GPIO_PORTA, 3)
#define HEATER_PWM GPIO(GPIO_PORTA, 4)
#define HELMHOLTZ_PWM GPIO(GPIO_PORTA, 5)
#define Q11_FREQ_COUNTER GPIO(GPIO_PORTA, 8)
#define Q12_FREQ_COUNTER GPIO(GPIO_PORTA, 9)
#define START_ADC GPIO(GPIO_PORTA, 10)
#define _DRDY_ADC GPIO(GPIO_PORTA, 11)
#define MOSI_ADC GPIO(GPIO_PORTA, 16)
#define SCK_ADC GPIO(GPIO_PORTA, 17)
#define MISO_ADC GPIO(GPIO_PORTA, 19)
#define _CS_ADC GPIO(GPIO_PORTA, 21)
#define _RESET_ADC GPIO(GPIO_PORTA, 22)
#define _PWDN_ADC GPIO(GPIO_PORTA, 23)
#define RED_LED GPIO(GPIO_PORTB, 10)
#define BLUE_LED GPIO(GPIO_PORTB, 11)
#define Q14_FREQ_COUNTER GPIO(GPIO_PORTB, 22)
#define Q13_FREQ_COUNTER GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
