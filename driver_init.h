/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_spi_m_sync.h>

#include <hal_pwm.h>
#include <hpl_tcc.h>
#include <tcc_lite.h>
#include <tcc_lite.h>

#include <hal_wdt.h>

extern struct spi_m_sync_descriptor SPI_0;

extern struct pwm_descriptor PWM_0;

extern struct wdt_descriptor WDT_0;

void SPI_0_PORT_init(void);
void SPI_0_CLOCK_init(void);
void SPI_0_init(void);

void PWM_0_PORT_init(void);
void PWM_0_CLOCK_init(void);
void PWM_0_init(void);

void TIMER_1_CLOCK_init(void);

void TIMER_1_PORT_init(void);

int8_t TIMER_1_init(void);

void TIMER_0_CLOCK_init(void);

void TIMER_0_PORT_init(void);

int8_t TIMER_0_init(void);

void WDT_0_CLOCK_init(void);
void WDT_0_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED