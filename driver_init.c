/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

struct spi_m_sync_descriptor SPI_0;

struct pwm_descriptor PWM_0;

struct wdt_descriptor WDT_0;

void SPI_0_PORT_init(void)
{

	gpio_set_pin_level(MOSI_ADC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOSI_ADC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOSI_ADC, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_level(SCK_ADC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SCK_ADC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SCK_ADC, PINMUX_PA17C_SERCOM1_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(MISO_ADC, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MISO_ADC,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(MISO_ADC, PINMUX_PA19C_SERCOM1_PAD3);
}

void SPI_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM1);
	SPI_0_PORT_init();
}

void PWM_0_PORT_init(void)
{

	gpio_set_pin_function(HEATER_PWM, PINMUX_PA04E_TCC0_WO0);

	gpio_set_pin_function(HELMHOLTZ_PWM, PINMUX_PA05E_TCC0_WO1);
}

void PWM_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC0);
	_gclk_enable_channel(TCC0_GCLK_ID, CONF_GCLK_TCC0_SRC);
}

void PWM_0_init(void)
{
	PWM_0_CLOCK_init();
	PWM_0_PORT_init();
	pwm_init(&PWM_0, TCC0, _tcc_get_pwm());
}

void TIMER_1_PORT_init(void)
{
}

void TIMER_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC1);
	_gclk_enable_channel(TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC);
}

void TIMER_0_PORT_init(void)
{
}

void TIMER_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC2);
	_gclk_enable_channel(TCC2_GCLK_ID, CONF_GCLK_TCC2_SRC);
}

void WDT_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, WDT);
	_gclk_enable_channel(WDT_GCLK_ID, CONF_GCLK_WDT_SRC);
}

void WDT_0_init(void)
{
	WDT_0_CLOCK_init();
	wdt_init(&WDT_0, WDT);
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA03

	gpio_set_pin_level(TCC1_out,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(TCC1_out, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(TCC1_out, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA08

	// Set pin direction to input
	gpio_set_pin_direction(Q11_FREQ_COUNTER, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Q11_FREQ_COUNTER,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_DOWN);

	gpio_set_pin_function(Q11_FREQ_COUNTER, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA09

	// Set pin direction to input
	gpio_set_pin_direction(Q12_FREQ_COUNTER, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Q12_FREQ_COUNTER,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_DOWN);

	gpio_set_pin_function(Q12_FREQ_COUNTER, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA10

	gpio_set_pin_level(START_ADC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(START_ADC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(START_ADC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA11

	// Set pin direction to input
	gpio_set_pin_direction(_DRDY_ADC, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(_DRDY_ADC,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(_DRDY_ADC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	gpio_set_pin_level(_CS_ADC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(_CS_ADC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(_CS_ADC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA22

	gpio_set_pin_level(_RESET_ADC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(_RESET_ADC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(_RESET_ADC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23

	gpio_set_pin_level(_PWDN_ADC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(_PWDN_ADC, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(_PWDN_ADC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10

	gpio_set_pin_level(RED_LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(RED_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RED_LED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11

	gpio_set_pin_level(BLUE_LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BLUE_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BLUE_LED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	// Set pin direction to input
	gpio_set_pin_direction(Q14_FREQ_COUNTER, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Q14_FREQ_COUNTER,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_DOWN);

	gpio_set_pin_function(Q14_FREQ_COUNTER, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB23

	// Set pin direction to input
	gpio_set_pin_direction(Q13_FREQ_COUNTER, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(Q13_FREQ_COUNTER,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_DOWN);

	gpio_set_pin_function(Q13_FREQ_COUNTER, GPIO_PIN_FUNCTION_OFF);

	SPI_0_init();

	PWM_0_init();

	TIMER_1_CLOCK_init();

	TIMER_1_PORT_init();

	TIMER_1_init();

	TIMER_0_CLOCK_init();

	TIMER_0_PORT_init();

	TIMER_0_init();

	WDT_0_init();
}
