#include "ultrasonic.h"
#include "tim.h"
#include <stdbool.h>

#define US_TIM        (&htim2)
#define US_CH         TIM_CHANNEL_2
#define US_ACTIVE_CH  HAL_TIM_ACTIVE_CHANNEL_2
#define US_FLAG_CC    TIM_FLAG_CC2
#define US_FLAG_CCOF  TIM_FLAG_CC2OF

#define US_MIN_US     100u    /* ~17 mm, shorter = noise */
#define US_MAX_US     25000u  /* ~4.3 m; no-target pulses (~38 ms) are rejected */
#define US_MAX_AGE_MS 250u    /* older results are reported as -1 */

typedef enum { US_IDLE, US_WAIT_RISE, US_WAIT_FALL, US_DONE } us_state_t;

static volatile us_state_t state = US_IDLE;
static volatile uint32_t rise_tick;   /* TIM2 count at rising edge */
static volatile uint32_t width_us;    /* echo high time */
static uint32_t ping_ms;              /* HAL tick when the pending ping was fired */
static bool started;

static inline void set_polarity(uint32_t pol) {
    __HAL_TIM_SET_CAPTUREPOLARITY(US_TIM, US_CH, pol);
}

HAL_StatusTypeDef Ultrasonic_Start(void)
{
    gpio_reset(US_TRIG);

    /* TIM2 sits on APB1: its clock is PCLK1, x2 when the APB1 prescaler != 1
     * (168 MHz SYSCLK, APB1 /4 -> 42 MHz PCLK1 -> 84 MHz TIM2). The .ioc value
     * of 72-1 assumes a 72 MHz part, so set a 1 MHz tick from the real clock. */
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) timclk *= 2u;
    if (timclk < 1000000u) return HAL_ERROR;

    US_TIM->Init.Prescaler = timclk / 1000000u - 1u;
    US_TIM->Init.Period    = 0xFFFFFFFFu;               /* TIM2 is 32-bit: wraps after ~71 min */
    __HAL_TIM_SET_PRESCALER(US_TIM, US_TIM->Init.Prescaler);
    __HAL_TIM_SET_AUTORELOAD(US_TIM, US_TIM->Init.Period);
    US_TIM->Instance->EGR = TIM_EGR_UG;                 /* load the prescaler now */
    __HAL_TIM_CLEAR_FLAG(US_TIM, TIM_FLAG_UPDATE | US_FLAG_CC | US_FLAG_CCOF);

    state = US_IDLE;
    set_polarity(TIM_INPUTCHANNELPOLARITY_RISING);
    if (HAL_TIM_IC_Start_IT(US_TIM, US_CH) != HAL_OK) return HAL_ERROR;
    started = true;
    return HAL_OK;
}

int32_t Ultrasonic_ReadMm(void)
{
    if (!started) return -1;

    /* Collect the previous ping's result and drop any unfinished capture.
     * IRQs off so the ISR can't change state between the read and the reset. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    us_state_t s = state;
    uint32_t w = width_us;
    state = US_IDLE;
    set_polarity(TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(US_TIM, US_FLAG_CC | US_FLAG_CCOF);
    __set_PRIMASK(primask);

    int32_t mm = -1;
    if (s == US_DONE && w >= US_MIN_US && w <= US_MAX_US
        && (HAL_GetTick() - ping_ms) <= US_MAX_AGE_MS)
        mm = (int32_t)((w * 343u) / 2000u);              /* 343 m/s, round trip */

    /* Fire the next ping, unless echo is still high from a long no-target
     * pulse (the sensor ignores triggers until it drops). The pin is in AF
     * mode but IDR still reads its level. */
    if (gpio_read(US_ECHO) == GPIO_PIN_RESET) {
        state = US_WAIT_RISE;                           /* arm before triggering */
        gpio_set(US_TRIG);
        uint32_t t0 = __HAL_TIM_GET_COUNTER(US_TIM);
        while ((__HAL_TIM_GET_COUNTER(US_TIM) - t0) < 12u) { }  /* >= 10 us */
        gpio_reset(US_TRIG);
        ping_ms = HAL_GetTick();
    }
    return mm;
}

/* TIM2 is the only input-capture timer, so this callback is defined here.
 * The ELCP motors use HAL_GPIO_EXTI_Callback (elcp_motor.c) and TIM8/TIM12 run
 * PWM without interrupts, so nothing else uses this callback. If another
 * input-capture timer is added, dispatch to it from here by htim->Instance. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2 || htim->Channel != US_ACTIVE_CH) return;

    uint32_t t = HAL_TIM_ReadCapturedValue(htim, US_CH);
    if (state == US_WAIT_RISE) {
        rise_tick = t;
        state = US_WAIT_FALL;
        set_polarity(TIM_INPUTCHANNELPOLARITY_FALLING);
    } else if (state == US_WAIT_FALL) {
        width_us = t - rise_tick;                        /* unsigned math handles wrap */
        state = US_DONE;
        set_polarity(TIM_INPUTCHANNELPOLARITY_RISING);
    }
    /* US_IDLE / US_DONE: stray edge, ignore */
}
