/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
#include "elcp_motor/elcp_motor.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static IR_Sensors ir;
static int32_t us_mm[ULTRASONIC_COUNT] = {-1, -1, -1};  /* latest raw reading per ultrasonic */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Startup log: each call adds a line and redraws the TFT immediately.
 * tft_update clears the buffer after drawing, so all lines are reprinted each time. */
static void boot_log(const char *msg) {
  static const char *lines[10];
  static unsigned count;
  if (count < 10) lines[count++] = msg;
  for (unsigned i = 0; i < count; i++) tft_prints(0, i, "%s", lines[i]);
  tft_update(0);
}

/* Fills the screen buffer with all sensor values (ST7735 pin-on-top: 16 x 10).
 * Row 0 is the title. The caller pushes it with tft_update, which clears the buffer. */
static void show_status(const char *title) {
  static const char *const ir_names[IR_COUNT] = {"LL", "LM", "RM", "RR"};
  tft_prints(0, 0, "%s", title);
  for (unsigned i = 0; i < IR_COUNT; i++) {            /* rows 1-4: IR raw, trigger, black */
    char state = !ir.ok[i] ? '?' : ir.black[i] ? '#' : '.';
    tft_prints(0, 1 + i, "%s %4d %4d %c", ir_names[i],
               (int)ir.raw[i], (int)ir.trigger[i], state);
  }
  for (unsigned i = 0; i < ULTRASONIC_COUNT; i++) {    /* rows 5-7: US1 front, US2, US3 */
    if (us_mm[i] < 0) tft_prints(0, 5 + i, "US%u NO ECHO", i + 1);
    else              tft_prints(0, 5 + i, "US%u %4d mm", i + 1, (int)us_mm[i]);
  }
  const ELCP_Motor_State_t *m1 = ELCP_Motor_GetState(MOTOR_J1);   /* rows 8-9: motors */
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  tft_prints(0, 8, "J1%+5d%%%+5drpm", (int)m1->current_speed_pct, (int)m1->rpm);
  tft_prints(0, 9, "J3%+5d%%%+5drpm", (int)m3->current_speed_pct, (int)m3->rpm);
}

/* Same screen during key 1 (which blocks the main loop), with the phase as the
 * title. Redraws every 100 ms; only changed characters are sent, so it is quick. */
static void key1_show(const char *phase) {
  static uint32_t last;
  if (HAL_GetTick() - last < 100) return;
  last = HAL_GetTick();
  IR_Update(&ir);
  ELCP_Motor_UpdatePPS(MOTOR_J1);
  ELCP_Motor_UpdatePPS(MOTOR_J3);
  show_status(phase);
  tft_update(0);
}

/* ---- Key 1 -------------------------------------------------------------------------
 * 1. START    1 s still; record US2-US3 (rear) = the robot's straight angle.
 * 2. FORWARD  line PID (PID_FWD) at BASE_SPEED, APPROACH_SPEED inside APPROACH_MM;
 *             stop when US1 (front) < US_STOP_MM twice in a row.
 * 3. PAUSE    1 s still, rear ultrasonics settle.
 * 4. SQUARE   turn on the spot until US2-US3 matches the start angle again.
 * 5. BACK     line PID (PID_BACK) at BACK_SPEED + rear-ultrasonic heading correction
 *             (keep the start angle); stop when US2/US3 < BACK_STOP_MM twice in a row,
 *             or at the encoder backstop.
 * Line reading (both ways): middle sensors first, outer only if both middle are off,
 * 3+ black (junction) = straight, line lost = keep steering toward the last side.
 * ------------------------------------------------------------------------------------ */
#define BASE_SPEED  25.0f  /* % before gearing; J1 gets 2x this */
#define J1_GAIN     2.0f   /* J1 needs ~2x J3 to drive straight */
#define J3_GAIN     1.0f
#define LINE_KP     -2.0f  /* steering toward the line (small steps); negative = J1 is the right wheel */
#define LINE_KD     -6.0f  /* damping: brakes the turn as the line comes back to centre. Same sign as LINE_KP */
#define LINE_KI     0.0f   /* forward I: off, forward already holds the line */
#define LINE_I_MAX  0.0f   /* cap on the forward I part, % */
#define LINE_DT_MS  5      /* steering update period, ms: small steps, often */
#define LINE_SLEW   1.0f   /* max steering change per update, % (5 ms -> 0 to full turn in 100 ms) */
#define LINE_TURN_MAX 8.0f /* steering limit: small, so it nearly drives straight */
#define US_STOP_MM  90     /* forward stops this close to the key holder (80-100 mm) */
#define APPROACH_MM 400    /* inside this, forward slows down for a precise stop (IR stays on) */
#define APPROACH_SPEED 15.0f /* forward speed inside APPROACH_MM */
#define TURN_PAUSE_MS 1000 /* standstill between forward and reverse */
#define BACK_SPEED  15.0f  /* slower on the way back */
#define BACK_STOP_MM 150   /* reverse stops this close to the back wall */
/* Reverse line following: the same PID as forward with its own gains. (Inverting both
 * the sensor reading and the steering cancels out, so it is the same function.) */
#define BACK_KP     -3.0f  /* reverse P; negative tested (+ circled off) */
#define BACK_KI     -4.0f  /* reverse I, same sign: builds up against a steady pull (motor imbalance) */
#define BACK_I_MAX  6.0f   /* cap on the reverse I part, %: stops it winding up */
#define BACK_KD     -12.0f /* reverse D, same sign as BACK_KP: damping against the growing drift */
#define BACK_SLEW   0.5f   /* reverse step per 5 ms: half of forward's 1.0, 0 to full in 60 ms */
#define BACK_TURN_MAX 8.0f /* reverse steering limit (line + ultrasonic together) */
#define BACK_US_K   0.1f   /* reverse: % steering per mm the rear US2-US3 difference is off the start angle */
#define BACK_US_MAX 5.0f   /* cap on that ultrasonic correction, % */
#define BACK_RANGE  1.5f   /* encoder backstop: reverse at most this x the forward distance */
#define ALIGN_SPEED 10.0f  /* % pivot speed while squaring up at the target */
#define ALIGN_SIGN  -1.0f  /* pivot direction; flip if it turns away from square (tested: -1) */
#define ALIGN_TOL_MM 10    /* US2-US3 within this of the start difference = square */
#define ALIGN_MAX_MS 3000  /* give up squaring after this */
#define US_EMA_A    0.5f   /* rear ultrasonic smoothing: 1 = none, smaller = smoother but slower */

static float line_err;     /* -2 = line far left .. +2 = line far right; kept if the line is lost */
/* Straight-only following: while a middle sensor sees the line, the outer sensors
 * are ignored, so a junction branch touching LL/RR can't steer it off the straight.
 * The outer sensors only steer when the line has slipped off both middle ones. */
static float line_hist[4]; /* last 4 errors, so D measures the change over 20 ms */
static unsigned line_hi;
static float line_turn;
static float line_i;       /* accumulated I part, % */
static uint32_t line_ms;

typedef struct { float kp, ki, kd, slew, turn_max, i_max; } line_pid_t;
static const line_pid_t PID_FWD  = {LINE_KP, LINE_KI, LINE_KD, LINE_SLEW, LINE_TURN_MAX, LINE_I_MAX};
static const line_pid_t PID_BACK = {BACK_KP, BACK_KI, BACK_KD, BACK_SLEW, BACK_TURN_MAX, BACK_I_MAX};

/* Fresh steering state, before each drive segment. */
static void line_reset(void) {
  line_err = line_turn = line_i = 0;
  line_hist[0] = line_hist[1] = line_hist[2] = line_hist[3] = 0;
  line_hi = 0;
  line_ms = 0;
}

/* Reads the IR and updates line_err. Returns false if no sensor sees the line.
 * Junction: 3+ black sensors reads as centred (straight), so a branch can't pull it
 * off. Every other reading is followed normally, straight away. */
static bool line_read(void) {
  IR_Update(&ir);
  bool ll = ir.black[IR_LEFTMOST], lm = ir.black[IR_LEFT_MIDDLE];
  bool rm = ir.black[IR_RIGHT_MIDDLE], rr = ir.black[IR_RIGHTMOST];
  if ((int)ll + (int)lm + (int)rm + (int)rr >= 3) line_err = 0.0f;  /* junction: straight */
  else if (lm || rm)       line_err = (float)rm - (float)lm;  /* -1, 0 (centred) or +1 */
  else if (ll && !rr) line_err = -2.0f;                  /* slipped far left */
  else if (rr && !ll) line_err = 2.0f;                   /* slipped far right */
  /* nothing, or only both outer sensors: keep the last value */
  return ll || lm || rm || rr;
}

/* PID line following for both directions (base > 0 forward, < 0 reverse), recalculated
 * every LINE_DT_MS. If the line is lost, line_err keeps its last value, so it keeps
 * steering toward the side the line was last seen on until it finds it again. */
static void line_drive(float base, const line_pid_t *c, float extra) {
  if (HAL_GetTick() - line_ms >= LINE_DT_MS) {
    line_ms = HAL_GetTick();
    line_read();
    float change = line_err - line_hist[line_hi];   /* over the last 4 updates = 20 ms */
    line_hist[line_hi] = line_err;
    line_hi = (line_hi + 1) % 4;

    line_i += c->ki * line_err * (LINE_DT_MS / 1000.0f);   /* I: % per second of error */
    if (line_i > c->i_max) line_i = c->i_max;
    if (line_i < -c->i_max) line_i = -c->i_max;

    float target = c->kp * line_err + line_i + c->kd * change;
    if (target > c->turn_max) target = c->turn_max;
    if (target < -c->turn_max) target = -c->turn_max;

    /* Move toward the target by at most slew per update: small, frequent steps. */
    if (target > line_turn + c->slew)      line_turn += c->slew;
    else if (target < line_turn - c->slew) line_turn -= c->slew;
    else                                   line_turn = target;
  }
  float turn = line_turn + extra;              /* line PID + extra (rear ultrasonics) */
  if (turn > c->turn_max) turn = c->turn_max;
  if (turn < -c->turn_max) turn = -c->turn_max;
  ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * (base + turn));
  ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * (base - turn));
}

/* Rear ultrasonics US2/US3, each smoothed by an EMA. rear_poll pings one of them
 * every 30 ms (they take turns, so they don't hear each other) and returns true
 * when a fresh valid reading arrived. A sensor's value counts for 300 ms. */
static float rear[2];                      /* mm, -1 = none yet */
static uint32_t rear_ms[2], rear_last;
static unsigned rear_next;

static void rear_reset(void) {
  rear[0] = rear[1] = -1;
  rear_ms[0] = rear_ms[1] = rear_last = HAL_GetTick();
  rear_next = 0;
}
static bool rear_poll(void) {
  if (HAL_GetTick() - rear_last < 30) return false;
  rear_last = HAL_GetTick();
  unsigned i = rear_next;
  rear_next ^= 1;
  int32_t d = Ultrasonic_ReadMm(1 + i);    /* 1 = US2, 2 = US3 */
  us_mm[1 + i] = d;                        /* raw value for the TFT */
  if (d < 0) return false;
  rear[i] = rear[i] < 0 ? (float)d : rear[i] + US_EMA_A * ((float)d - rear[i]);
  rear_ms[i] = rear_last;
  return true;
}
static bool rear_ok(unsigned i) { return rear[i] >= 0 && HAL_GetTick() - rear_ms[i] < 300; }

static void key1(void) {
  line_reset();
  uint32_t wait = HAL_GetTick();             /* 1 s after the press before moving */
  rear_reset();
  while (HAL_GetTick() - wait < 1000) {
    rear_poll();
    key1_show("K1 START");
  }
  /* US2-US3 at the start, where the robot is placed straight. The two sensors do not
   * read equal when square (mounting, wall), so squaring up aims for this difference. */
  float square_diff = (rear_ok(0) && rear_ok(1)) ? rear[0] - rear[1] : 0;

  const ELCP_Motor_State_t *m1 = ELCP_Motor_GetState(MOTOR_J1);
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  int32_t s1 = m1->pulse_count, s3 = m3->pulse_count;   /* encoder counts at the start */

  /* Forward, following the line, until 2 ultrasonic readings in a row are closer than US_STOP_MM. */
  int close = 0;
  uint32_t last_us = 0;
  float fwd_speed = BASE_SPEED;
  while (close < 2) {
    line_drive(fwd_speed, &PID_FWD, 0);
    if (HAL_GetTick() - last_us >= 60) {
      last_us = HAL_GetTick();
      int32_t d = Ultrasonic_ReadMm(0);   /* US1, front */
      us_mm[0] = d;
      close = (d >= 0 && d < US_STOP_MM) ? close + 1 : 0;
      if (d >= 0 && d < APPROACH_MM) fwd_speed = APPROACH_SPEED;  /* slow final approach, stays slow */
    }
    key1_show("K1 FORWARD");
  }
  /* Full stop before reversing, so the forward motion has died out. */
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  int32_t f1 = m1->pulse_count - s1, f3 = m3->pulse_count - s3;  /* forward pulses per wheel */
  /* Pause standing still, but keep pinging the rear ultrasonics so their smoothed
   * values are settled before squaring up starts (a plain HAL_Delay sends no pings). */
  rear_reset();
  uint32_t pause = HAL_GetTick();
  while (HAL_GetTick() - pause < TURN_PAUSE_MS) {
    rear_poll();
    key1_show("K1 PAUSE");
  }

  /* Square up on the spot: pivot slowly until US2 and US3 agree within
   * ALIGN_TOL_MM for 3 fresh readings in a row (or give up after ALIGN_MAX_MS). */
  uint32_t t0 = HAL_GetTick();
  int square = 0;
  while (square < 3 && HAL_GetTick() - t0 < ALIGN_MAX_MS) {
    bool fresh = rear_poll();
    float pivot = 0;
    if (rear_ok(0) && rear_ok(1)) {
      float diff = (rear[0] - rear[1]) - square_diff;   /* 0 = same angle as at the start */
      bool inside = diff <= ALIGN_TOL_MM && diff >= -ALIGN_TOL_MM;
      if (fresh) square = inside ? square + 1 : 0;
      if (!inside) pivot = (diff > 0 ? ALIGN_SPEED : -ALIGN_SPEED) * ALIGN_SIGN;
    }
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * pivot);   /* wheels opposite: turn in place */
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * -pivot);
    key1_show("K1 SQUARE UP");
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);

  /* Reverse with the same line following as forward, with the reverse PID gains.
   * Stops when the rear ultrasonics read under BACK_STOP_MM twice in a row, or at the
   * encoder backstop of BACK_RANGE x the forward distance. */
  int32_t backstop = (int32_t)(BACK_RANGE * (float)(f1 + f3));
  int32_t b1 = m1->pulse_count, b3 = m3->pulse_count;
  int near = 0;
  float us_turn = 0;                          /* rear-ultrasonic heading correction */
  line_reset();
  while (near < 2) {
    int32_t g1 = b1 - m1->pulse_count, g3 = b3 - m3->pulse_count;  /* pulses gone backward */
    if (backstop > 0 && g1 + g3 >= backstop) break;

    /* Keep the same angle to the back wall as at the start (same steering direction
     * as squaring up). If a rear sensor is missing, keep the last correction. */
    if (rear_ok(0) && rear_ok(1)) {
      us_turn = ALIGN_SIGN * BACK_US_K * ((rear[0] - rear[1]) - square_diff);
      if (us_turn > BACK_US_MAX) us_turn = BACK_US_MAX;
      if (us_turn < -BACK_US_MAX) us_turn = -BACK_US_MAX;
    }
    line_drive(-BACK_SPEED, &PID_BACK, us_turn);

    if (rear_poll()) {                       /* count only fresh readings */
      bool ok2 = rear_ok(0), ok3 = rear_ok(1);
      float dist = (ok2 && ok3) ? (rear[0] + rear[1]) / 2 : ok2 ? rear[0] : rear[1];
      near = dist < BACK_STOP_MM ? near + 1 : 0;
    }
    key1_show("K1 BACK");
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Starts TIM12 CH1 / TIM8 CH2 PWM with both motors stopped (0 % duty). */
  ELCP_Motor_Init(MOTOR_J1);
  ELCP_Motor_Init(MOTOR_J3);
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);

  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  boot_log("SENSOR STARTUP");
  boot_log("TFT OK");

  IR_Init(&ir, IR_DEFAULT_TRIGGER);
  boot_log((ir.ok[0] && ir.ok[1] && ir.ok[2] && ir.ok[3]) ? "IR OK" : "IR ADC FAIL");

  boot_log(Ultrasonic_Start() == HAL_OK ? "US x3 OK" : "US FAIL");

  HAL_Delay(1500);   /* keep the startup log on screen briefly */
  unsigned us_next = 0;
  uint32_t last_us = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Read sensors */
    IR_Update(&ir);
    /* Ultrasonics take turns, one every 30 ms (each every 90 ms), so they don't
     * hear each other's echoes. */
    if (HAL_GetTick() - last_us >= 30) {
      last_us = HAL_GetTick();
      us_mm[us_next] = Ultrasonic_ReadMm(us_next);
      us_next = (us_next + 1) % ULTRASONIC_COUNT;
    }
    if (btn_read(BTN1)) key1();
    if (btn_read(BTN2)) {
      ELCP_Motor_SetSpeed(MOTOR_J1, 50.0f);    /* forward */
      ELCP_Motor_SetSpeed(MOTOR_J3, 25.0f);
      HAL_Delay(5000);
      ELCP_Motor_Stop(MOTOR_J1);               /* stop */
      ELCP_Motor_Stop(MOTOR_J3);
      HAL_Delay(1000);
      ELCP_Motor_SetSpeed(MOTOR_J1, -50.0f);   /* backward */
      ELCP_Motor_SetSpeed(MOTOR_J3, -25.0f);
      HAL_Delay(5000);
      ELCP_Motor_Stop(MOTOR_J1);
      ELCP_Motor_Stop(MOTOR_J3);
    }

    ELCP_Motor_UpdatePPS(MOTOR_J1);          /* refreshes pps/rpm once per second */
    ELCP_Motor_UpdatePPS(MOTOR_J3);

    show_status("IR  RAW TRIG B");
    tft_update(50);                          /* push to the TFT every 50 ms */
  }
  

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Remove PWM on a HAL fatal error; do not depend on initialized motor handles. */
  if (__HAL_RCC_TIM8_IS_CLK_ENABLED()) TIM8->CCR2 = 0;
  if (__HAL_RCC_TIM12_IS_CLK_ENABLED()) TIM12->CCR1 = 0;
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
