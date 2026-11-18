#include "ultrasonic.h"
#include <stdbool.h>
static bool dwt_ready;
static uint32_t cyc_per_us;
static inline uint32_t cyc(void) { return DWT->CYCCNT; }
HAL_StatusTypeDef Ultrasonic_Start(void) {
    gpio_reset(US_TRIG);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cyc_per_us=HAL_RCC_GetHCLKFreq()/1000000u;
    dwt_ready=cyc_per_us!=0;
    return dwt_ready?HAL_OK:HAL_ERROR;
}
int32_t Ultrasonic_ReadMm(void)
{
    if (!dwt_ready || !cyc_per_us) return -1;
    if (gpio_read(US_ECHO) == GPIO_PIN_SET) return -1;        /* echo stuck high */
    gpio_set(US_TRIG);
    uint32_t t0 = cyc();
    while ((cyc() - t0) < 15u * cyc_per_us) { }
    gpio_reset(US_TRIG);
    uint32_t t1 = cyc(), tmax = 6000u * cyc_per_us;
    while (gpio_read(US_ECHO) == GPIO_PIN_RESET) { if ((cyc() - t1) > tmax) return -1; }
    uint32_t r0 = cyc();
    while (gpio_read(US_ECHO) == GPIO_PIN_SET) { if ((cyc() - t1) > tmax) return -1; }
    uint32_t us = (cyc() - r0) / cyc_per_us;
    if (us < 100u) return -1;
    return (int32_t)((us * 343u) / 2000u);                    /* mm */
}
