#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

/* HC-SR04 style sensor.
 *   PA0 = US_TRIG  plain GPIO output
 *   PA1 = US_ECHO  TIM2_CH2, input capture (direct TI), TIM2 global interrupt
 *
 * The echo pulse width is timestamped by TIM2 hardware, so other interrupts
 * (e.g. the ELCP motor FB EXTIs) delaying the ISR do not affect accuracy. */

/* Configures TIM2 for 1 us ticks (from the real APB1 timer clock) and starts
 * channel 2 input capture with interrupt. Call once after MX_TIM2_Init(). */
HAL_StatusTypeDef Ultrasonic_Start(void);

/* Non-blocking. Returns the distance in mm from the PREVIOUS ping (or -1 if it
 * got no/invalid echo, or is older than 250 ms), then fires a new ping.
 * Call every >= 60 ms; the first call after start returns -1. */
int32_t Ultrasonic_ReadMm(void);

#endif
