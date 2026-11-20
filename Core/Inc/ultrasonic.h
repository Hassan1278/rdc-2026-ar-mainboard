#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

/* Three HC-SR04 style sensors. Each has a plain GPIO trigger and its echo on an
 * input-capture channel (direct TI) with that timer's interrupt (pins from the .ioc):
 *   index 0  US   TRIG PA0   ECHO PA1   TIM2_CH2   (front, used by key 1)
 *   index 1  US1  TRIG PB8   ECHO PB9   TIM4_CH4
 *   index 2  US2  TRIG PB11  ECHO PB10  TIM2_CH3
 *
 * The echo pulse width is timestamped by timer hardware, so other interrupts
 * (e.g. the ELCP motor FB EXTIs) delaying the ISR do not affect accuracy. */
#define ULTRASONIC_COUNT 3

/* Configures TIM2/TIM4 for 1 us ticks (from the real APB1 timer clock) and starts
 * input capture with interrupt on each sensor's channel.
 * Call once after MX_TIM2_Init and MX_TIM4_Init. */
HAL_StatusTypeDef Ultrasonic_Start(void);

/* Non-blocking, for sensor i (0..2). Returns the distance in mm from that
 * sensor's PREVIOUS ping (or -1 if it got no/invalid echo, or is older than
 * 250 ms), then fires a new ping. Call each sensor every >= 60 ms; the first
 * call after start returns -1. Avoid pinging sensors that face the same way at
 * the same moment, or they hear each other's echoes. */
int32_t Ultrasonic_ReadMm(unsigned i);

#endif
