#ifndef ULTRASONIC_GPIO_H
#define ULTRASONIC_GPIO_H

#include "main.h"
#include <stdint.h>

/* PA0 = US_TRIG (output), PA1 = US_ECHO (input). Timing uses the DWT cycle counter,
 * so no timer is needed. */
HAL_StatusTypeDef Ultrasonic_Start(void);
/* Blocking measurement (max ~6 ms). Returns distance in mm, or -1 on no/invalid echo.
 * Leave at least ~60 ms between calls. */
int32_t Ultrasonic_ReadMm(void);

#endif
