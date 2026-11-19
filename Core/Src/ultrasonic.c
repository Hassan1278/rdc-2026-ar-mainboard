#include "ultrasonic.h"
#include "tim.h"
#include <stdbool.h>

#define US_CH         TIM_CHANNEL_2           /* every echo is on channel 2 */
#define US_ACTIVE_CH  HAL_TIM_ACTIVE_CHANNEL_2
#define US_FLAG_CC    TIM_FLAG_CC2
#define US_FLAG_CCOF  TIM_FLAG_CC2OF

#define US_MIN_US     100u    /* ~17 mm, shorter = noise */
#define US_MAX_US     25000u  /* ~4.3 m; no-target pulses (~38 ms) are rejected */
#define US_MAX_AGE_MS 250u    /* older results are reported as -1 */

typedef enum { US_IDLE, US_WAIT_RISE, US_WAIT_FALL, US_DONE } us_state_t;

typedef struct {
    TIM_HandleTypeDef *tim;
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
    uint32_t mask;                 /* counter width: 0xFFFFFFFF (TIM2) or 0xFFFF (TIM3/TIM4) */
    volatile us_state_t state;
    volatile uint32_t rise_tick;   /* timer count at rising edge */
    volatile uint32_t width_us;    /* echo high time */
    uint32_t ping_ms;              /* HAL tick when the pending ping was fired */
} us_sensor_t;

static us_sensor_t us[ULTRASONIC_COUNT] = {
    {&htim2, US_TRIG_GPIO_Port,  US_TRIG_Pin,  US_ECHO_GPIO_Port,  US_ECHO_Pin},
    {&htim4, US_TRIG2_GPIO_Port, US_TRIG2_Pin, US_ECHO2_GPIO_Port, US_ECHO2_Pin},
    {&htim3, US_TRIG3_GPIO_Port, US_TRIG3_Pin, US_ECHO3_GPIO_Port, US_ECHO3_Pin},
};
static bool started;

static inline void set_polarity(us_sensor_t *s, uint32_t pol) {
    __HAL_TIM_SET_CAPTUREPOLARITY(s->tim, US_CH, pol);
}

HAL_StatusTypeDef Ultrasonic_Start(void)
{
    /* TIM2/3/4 sit on APB1: their clock is PCLK1, x2 when the APB1 prescaler != 1
     * (168 MHz SYSCLK, APB1 /4 -> 42 MHz PCLK1 -> 84 MHz timers). The .ioc value
     * of 72-1 assumes a 72 MHz part, so set a 1 MHz tick from the real clock. */
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) timclk *= 2u;
    if (timclk < 1000000u) return HAL_ERROR;

    for (unsigned i = 0; i < ULTRASONIC_COUNT; i++) {
        us_sensor_t *s = &us[i];
        HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

        /* TIM2 is 32-bit (wraps after ~71 min); TIM3/TIM4 are 16-bit (wrap every
         * 65 ms, longer than any valid echo, so masked differences stay correct). */
        s->mask = IS_TIM_32B_COUNTER_INSTANCE(s->tim->Instance) ? 0xFFFFFFFFu : 0xFFFFu;
        s->tim->Init.Prescaler = timclk / 1000000u - 1u;
        s->tim->Init.Period    = s->mask;
        __HAL_TIM_SET_PRESCALER(s->tim, s->tim->Init.Prescaler);
        __HAL_TIM_SET_AUTORELOAD(s->tim, s->tim->Init.Period);
        s->tim->Instance->EGR = TIM_EGR_UG;             /* load the prescaler now */
        __HAL_TIM_CLEAR_FLAG(s->tim, TIM_FLAG_UPDATE | US_FLAG_CC | US_FLAG_CCOF);

        s->state = US_IDLE;
        set_polarity(s, TIM_INPUTCHANNELPOLARITY_RISING);
        if (HAL_TIM_IC_Start_IT(s->tim, US_CH) != HAL_OK) return HAL_ERROR;
    }
    started = true;
    return HAL_OK;
}

int32_t Ultrasonic_ReadMm(unsigned i)
{
    if (!started || i >= ULTRASONIC_COUNT) return -1;
    us_sensor_t *s = &us[i];

    /* Collect the previous ping's result and drop any unfinished capture.
     * IRQs off so the ISR can't change state between the read and the reset. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    us_state_t st = s->state;
    uint32_t w = s->width_us;
    s->state = US_IDLE;
    set_polarity(s, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(s->tim, US_FLAG_CC | US_FLAG_CCOF);
    __set_PRIMASK(primask);

    int32_t mm = -1;
    if (st == US_DONE && w >= US_MIN_US && w <= US_MAX_US
        && (HAL_GetTick() - s->ping_ms) <= US_MAX_AGE_MS)
        mm = (int32_t)((w * 343u) / 2000u);              /* 343 m/s, round trip */

    /* Fire the next ping, unless echo is still high from a long no-target
     * pulse (the sensor ignores triggers until it drops). The pin is in AF
     * mode but IDR still reads its level. */
    if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
        s->state = US_WAIT_RISE;                        /* arm before triggering */
        HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
        uint32_t t0 = __HAL_TIM_GET_COUNTER(s->tim);
        while (((__HAL_TIM_GET_COUNTER(s->tim) - t0) & s->mask) < 12u) { }  /* >= 10 us */
        HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
        s->ping_ms = HAL_GetTick();
    }
    return mm;
}

/* TIM2/3/4 are the only input-capture timers, so this callback is defined here
 * and dispatches by timer. The ELCP motors use HAL_GPIO_EXTI_Callback
 * (elcp_motor.c) and TIM8/TIM12 run PWM without interrupts. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel != US_ACTIVE_CH) return;

    for (unsigned i = 0; i < ULTRASONIC_COUNT; i++) {
        us_sensor_t *s = &us[i];
        if (s->tim != htim) continue;

        uint32_t t = HAL_TIM_ReadCapturedValue(htim, US_CH);
        if (s->state == US_WAIT_RISE) {
            s->rise_tick = t;
            s->state = US_WAIT_FALL;
            set_polarity(s, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else if (s->state == US_WAIT_FALL) {
            s->width_us = (t - s->rise_tick) & s->mask;  /* masked math handles wrap */
            s->state = US_DONE;
            set_polarity(s, TIM_INPUTCHANNELPOLARITY_RISING);
        }
        /* US_IDLE / US_DONE: stray edge, ignore */
        return;
    }
}
