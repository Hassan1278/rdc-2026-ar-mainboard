#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

/* Three HC-SR04 style sensors. Each has a plain GPIO trigger and its echo on
 * channel 2 input capture (direct TI) of its own timer, with that timer's interrupt:
 *   index 0  US1  TRIG PA0  ECHO PA1  TIM2_CH2   (front)
 *   index 1  US2  TRIG PB6  ECHO PB7  TIM4_CH2
 *   index 2  US3  TRIG PB4  ECHO PB5  TIM3_CH2
 *
 * The echo pulse width is timestamped by timer hardware, so other interrupts
 * (e.g. the ELCP motor FB EXTIs) delaying the ISR do not affect accuracy. */
#define ULTRASONIC_COUNT 3

/* Configures TIM2/TIM3/TIM4 for 1 us ticks (from the real APB1 timer clock) and
 * starts channel 2 input capture with interrupt on each.
 * Call once after MX_TIM2_Init, MX_TIM3_Init and MX_TIM4_Init. */
HAL_StatusTypeDef Ultrasonic_Start(void);

/* Non-blocking, for sensor i (0..2). Returns the distance in mm from that
 * sensor's PREVIOUS ping (or -1 if it got no/invalid echo, or is older than
 * 250 ms), then fires a new ping. Call each sensor every >= 60 ms; the first
 * call after start returns -1. Avoid pinging sensors that face the same way at
 * the same moment, or they hear each other's echoes. */
int32_t Ultrasonic_ReadMm(unsigned i);

#endif
