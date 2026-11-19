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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
#include "elcp_motor/elcp_motor.h"
#include "ir_sensor.h"
#include "tof.h"
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
  static const char *const us_names[ULTRASONIC_COUNT] = {"US ", "US1", "US2"};  /* .ioc labels */
  for (unsigned i = 0; i < ULTRASONIC_COUNT; i++) {    /* rows 5-7: US (front), US1, US2 */
    if (us_mm[i] < 0) tft_prints(0, 5 + i, "%s NO ECHO", us_names[i]);
    else              tft_prints(0, 5 + i, "%s %4d mm", us_names[i], (int)us_mm[i]);
  }
  int32_t tof = ToF_DistanceMm();                      /* row 8: ToF */
  const ToF_Data *td = ToF_Get();
  if (tof >= 0)                tft_prints(0, 8, "TOF %4d mm", (int)tof);
  else if (td->status != TOF_OK) tft_prints(0, 8, "TOF %s", ToF_StatusText(td->status));
  else                         tft_prints(0, 8, "TOF ---- RS%u", td->range_status);
  const ELCP_Motor_State_t *m1 = ELCP_Motor_GetState(MOTOR_J1);   /* row 9: motors, % */
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  tft_prints(0, 9, "J1%+4d%% J3%+4d%%", (int)m1->current_speed_pct, (int)m3->current_speed_pct);
}

/* Same screen during key 1 (which blocks the main loop), with the phase as the
 * title. Redraws every 100 ms; only changed characters are sent, so it is quick. */
static int32_t pulses_shown = -1;          /* turn pulses, shown top right; -1 = none */

static void key1_show(const char *phase) {
  static uint32_t last;
  if (HAL_GetTick() - last < 100) return;
  last = HAL_GetTick();
  IR_Update(&ir);
  ToF_Update();                              /* never blocks */
  ELCP_Motor_UpdatePPS(MOTOR_J1);
  ELCP_Motor_UpdatePPS(MOTOR_J3);
  show_status(phase);
  if (pulses_shown >= 0) tft_prints(11, 0, "%5d", (int)pulses_shown);
  tft_update(0);
}

/* ---- Key 1 -------------------------------------------------------------------------
 * 1. START    1 s still.
 * 2. TO KEY   line PID at BASE_SPEED, APPROACH_SPEED inside APPROACH_MM;
 *             stop when US1 (front) < KEY_STOP_MM (160 mm) twice in a row. Straight line, so
 *             steering is capped at LINE_TURN_RATIO of the speed (no junctions).
 * 3. PAUSE    1 s still.
 * 4. KEY OFF  turn right RIGHT_DEG (60 deg), then pull back a little.
 * 5. TURN     right on the spot; a line seen inside TURN_WIN_MIN..TURN_WIN_MAX pulses
 *             is the straight path back: stop and follow it. Lines outside are ignored.
 * 6. RETURN   same line PID and steering cap as step 2, until all 4 IR see black (T-line),
 *             then straight ahead 2:1 into the start zone, stop at GOAL_STOP_MM (90 mm).
 * Line reading: middle sensors first, outer only if both middle are off,
 * 3+ black (junction) = straight, line lost = keep steering toward the last side.
 * ------------------------------------------------------------------------------------ */
#define BASE_SPEED  25.0f  /* % before gearing; J1 gets 2x this */
#define J1_GAIN     2.0f   /* J1 needs ~2x J3 to drive straight */
#define J3_GAIN     1.0f
#define LINE_KP     -2.0f  /* steering toward the line (small steps); negative = J1 is the right wheel */
#define LINE_KD     -6.0f  /* damping: brakes the turn as the line comes back to centre. Same sign as LINE_KP */
#define LINE_DT_MS  5      /* steering update period, ms: small steps, often */
#define LINE_SLEW   1.0f   /* max steering change per update, % (5 ms -> 0 to full turn in 100 ms) */
#define KEY_STOP_MM 160    /* forward leg: key detected, stop this far from the key holder */
#define GOAL_STOP_MM 90    /* return leg: after the T, drive straight 2:1 and stop this
                            * far from the wall of the start zone (front ultrasonic) */
#define GOAL_MAX_MS 8000   /* safety: stop the straight drive after this if US1 never trips */
#define GOAL_IGNORE_MS 500 /* return leg: ignore a T this soon after starting */
#define RETURN_MAX_MS 30000   /* safety: stop the return leg if no T is ever found */
#define LINE_TURN_RATIO 0.15f /* both legs are straight: steering at most 15% of the speed,
                              * so the two wheels can't differ enough to take a junction.
                              * Lower = straighter, but slower to correct a drift */
#define APPROACH_MM 400    /* inside this, slow down for a precise stop (IR stays on) */
#define APPROACH_SPEED 15.0f /* speed inside APPROACH_MM */
#define TURN_PAUSE_MS 1000 /* standstill before the turn */
#define TURN_SPEED  12.0f  /* % pivot speed for the turn */
#define TURN_MAX_MS 20000  /* safety only (e.g. encoder unplugged): TURN_WIN_MAX ends the turn */
#define TURN_WIN_MIN 200   /* turn-around window, encoder pulses of turning: the straight */
#define TURN_WIN_MAX 400   /* path back is the only line in here. CALIBRATE from the
                            * end-of-run screen: put the path's count inside, the junction
                            * branch's outside */
#define TURN_LOG_MAX 6     /* lines logged during the turn-around */
#define SETTLE_SPEED 10.0f /* % speed turning back onto the line after coasting past it */
#define SETTLE_MAX_MS 1500 /* give up turning back after this (then line following searches) */
#define RIGHT_DEG   60.0f  /* at the key: turn right this far */
#define PIVOT_SPEED 12.0f  /* % speed for that turn */
#define PIVOT_PULSES_PER_DEG 1.0f /* encoder pulses per degree of turning on the spot: CALIBRATE */
#define PULL_SPEED  15.0f  /* % speed for the small pullback before turning */
#define PULL_MS     1000   /* how long to pull back before turning */

static float line_err;     /* -2 = line far left .. +2 = line far right; kept if the line is lost */
static float line_hist[4]; /* last 4 errors, so D measures the change over 20 ms */
static unsigned line_hi;
static float line_turn;
static uint32_t line_ms;

/* Fresh steering state, before each drive. */
static void line_reset(void) {
  line_err = line_turn = 0;
  line_hist[0] = line_hist[1] = line_hist[2] = line_hist[3] = 0;
  line_hi = 0;
  line_ms = 0;
}

/* Reads the IR and updates line_err. Middle sensors first: the outer ones only count
 * when the line has left both middle ones, so a junction branch touching LL/RR can't
 * steer it. 3+ black (junction) reads as straight. Nothing seen: keep the last value. */
static void line_read(void) {
  IR_Update(&ir);
  bool ll = ir.black[IR_LEFTMOST], lm = ir.black[IR_LEFT_MIDDLE];
  bool rm = ir.black[IR_RIGHT_MIDDLE], rr = ir.black[IR_RIGHTMOST];
  if ((int)ll + (int)lm + (int)rm + (int)rr >= 3) line_err = 0.0f;  /* junction: straight */
  else if (lm || rm)  line_err = (float)rm - (float)lm;  /* -1, 0 (centred) or +1 */
  else if (ll && !rr) line_err = -2.0f;                  /* slipped far left */
  else if (rr && !ll) line_err = 2.0f;                   /* slipped far right */
}

/* PD line following, recalculated every LINE_DT_MS. */
static void line_drive(float base, float turn_max) {
  if (HAL_GetTick() - line_ms >= LINE_DT_MS) {
    line_ms = HAL_GetTick();
    line_read();
    float change = line_err - line_hist[line_hi];   /* over the last 4 updates = 20 ms */
    line_hist[line_hi] = line_err;
    line_hi = (line_hi + 1) % 4;

    float target = LINE_KP * line_err + LINE_KD * change;
    if (target > turn_max) target = turn_max;
    if (target < -turn_max) target = -turn_max;

    /* Move toward the target by at most LINE_SLEW per update: small, frequent steps. */
    if (target > line_turn + LINE_SLEW)      line_turn += LINE_SLEW;
    else if (target < line_turn - LINE_SLEW) line_turn -= LINE_SLEW;
    else                                     line_turn = target;
  }
  ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * (base + line_turn));
  ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * (base - line_turn));
}

/* Turn on the spot by deg degrees (+ = left, - = right), measured with the encoders.
 * Left = J1 (right wheel) forward, J3 back. 3 s safety limit. */
static void pivot_deg(float deg, const char *phase) {
  const ELCP_Motor_State_t *m1 = ELCP_Motor_GetState(MOTOR_J1);
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  int32_t p1 = m1->pulse_count, p3 = m3->pulse_count;
  float dir = deg > 0 ? 1.0f : -1.0f;
  int32_t target = (int32_t)(dir * deg * PIVOT_PULSES_PER_DEG);
  uint32_t t = HAL_GetTick();
  while (HAL_GetTick() - t < 3000) {
    int32_t turned = (int32_t)dir * ((m1->pulse_count - p1) - (m3->pulse_count - p3));
    pulses_shown = turned;
    if (turned >= target) break;
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * dir * PIVOT_SPEED);
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * -dir * PIVOT_SPEED);
    key1_show(phase);
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);
}

/* Forward leg: follow the straight line to the key holder. Slow down inside
 * APPROACH_MM, stop when the front ultrasonic US1 reads under KEY_STOP_MM twice in a
 * row. Steering is capped at LINE_TURN_RATIO of the speed, so a junction branch can't
 * pull it off the straight. */
static void drive_to_wall(const char *phase) {
  line_reset();
  int close = 0;
  uint32_t last_us = 0;
  float speed = BASE_SPEED;
  while (close < 2) {
    line_drive(speed, LINE_TURN_RATIO * speed);
    if (HAL_GetTick() - last_us >= 60) {
      last_us = HAL_GetTick();
      int32_t d = Ultrasonic_ReadMm(0);   /* US1, front */
      us_mm[0] = d;
      close = (d >= 0 && d < KEY_STOP_MM) ? close + 1 : 0;
      if (d >= 0 && d < APPROACH_MM) speed = APPROACH_SPEED;  /* slow final approach, stays slow */
    }
    key1_show(phase);
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
}

/* Return leg: follow the line (same steering cap as the forward leg) until all 4 IR see black
 * on two fresh readings in a row: the T-line at the goal. Then drive straight ahead,
 * no steering (J1:J3 = 2:1 like any straight run), into the start zone: slow down
 * inside APPROACH_MM, stop when US1 reads under GOAL_STOP_MM twice in a row.
 * start_err: which side the line is on if no sensor sees it at the start
 * (-1 left, 0 straight ahead, +1 right); a sensor seeing the line overrides it. */
static void drive_to_goal(const char *phase, float start_err) {
  line_reset();
  line_err = start_err;
  for (unsigned i = 0; i < 4; i++) line_hist[i] = start_err;   /* no D kick at start */
  uint32_t t0 = HAL_GetTick(), seen_read = 0;
  int t_hits = 0;
  while (HAL_GetTick() - t0 < RETURN_MAX_MS) {
    line_drive(BASE_SPEED, LINE_TURN_RATIO * BASE_SPEED);   /* same as the forward leg */
    if (line_ms != seen_read) {              /* a fresh IR reading (every LINE_DT_MS) */
      seen_read = line_ms;
      bool all = ir.black[IR_LEFTMOST] && ir.black[IR_LEFT_MIDDLE] &&
                 ir.black[IR_RIGHT_MIDDLE] && ir.black[IR_RIGHTMOST];
      t_hits = (all && HAL_GetTick() - t0 >= GOAL_IGNORE_MS) ? t_hits + 1 : 0;
      if (t_hits >= 2) break;                /* T-line: goal */
    }
    key1_show(phase);
  }
  /* Past the T: straight ahead at the plain 2:1 ratio into the start zone, stopped by
   * the front ultrasonic at GOAL_STOP_MM. GOAL_MAX_MS is a safety limit only. */
  uint32_t t = HAL_GetTick(), last_us = 0;
  int close = 0;
  float speed = BASE_SPEED;
  while (close < 2 && HAL_GetTick() - t < GOAL_MAX_MS) {
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * speed);
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * speed);
    if (HAL_GetTick() - last_us >= 60) {
      last_us = HAL_GetTick();
      int32_t d = Ultrasonic_ReadMm(0);      /* US1, front */
      us_mm[0] = d;
      close = (d >= 0 && d < GOAL_STOP_MM) ? close + 1 : 0;
      if (d >= 0 && d < APPROACH_MM) speed = APPROACH_SPEED;  /* slow final approach */
    }
    key1_show("K1 GOAL");
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
}

static void key1(void) {
  pulses_shown = -1;
  uint32_t t = HAL_GetTick();                /* 1 s after the press before moving */
  while (HAL_GetTick() - t < 1000) key1_show("K1 START");

  drive_to_wall("K1 TO KEY");

  t = HAL_GetTick();                         /* stand still at the key */
  while (HAL_GetTick() - t < TURN_PAUSE_MS) key1_show("K1 PAUSE");

  /* Turn right RIGHT_DEG degrees at the key. */
  pulses_shown = 0;
  pivot_deg(-RIGHT_DEG, "K1 RIGHT");

  /* Small pullback before turning around. */
  t = HAL_GetTick();
  while (HAL_GetTick() - t < PULL_MS) {
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * -PULL_SPEED);
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * -PULL_SPEED);
    key1_show("K1 PULL");
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);

  /* Turn around RIGHT on the spot (J1 back, J3 forward). The straight path back is the
   * only line inside the window TURN_WIN_MIN..TURN_WIN_MAX encoder pulses of turning:
   * lines outside it (the junction branch) are ignored. A middle sensor seeing black
   * inside the window = the path: stop and follow it. The window is measured with the
   * encoders, so battery level doesn't move it. Reaching TURN_WIN_MAX also stops the
   * turn, so it never swings on to another line.
   * Every line met (a middle sensor going white -> black) is logged with its pulse
   * count and shown after the run, to set the window. */
  const ELCP_Motor_State_t *m1 = ELCP_Motor_GetState(MOTOR_J1);
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  int32_t p1 = m1->pulse_count, p3 = m3->pulse_count;
  int32_t turned = 0, lines_at[TURN_LOG_MAX];
  unsigned lines = 0;
  bool was_middle = true;                    /* starts near the line: no edge yet */
  t = HAL_GetTick();
  while (HAL_GetTick() - t < TURN_MAX_MS && turned < TURN_WIN_MAX) {
    IR_Update(&ir);
    bool middle = ir.black[IR_LEFT_MIDDLE] || ir.black[IR_RIGHT_MIDDLE];
    turned = (m3->pulse_count - p3) - (m1->pulse_count - p1);
    if (middle && !was_middle) {             /* log every line met */
      if (lines < TURN_LOG_MAX) lines_at[lines] = turned;
      lines++;
    }
    was_middle = middle;
    if (middle && turned >= TURN_WIN_MIN) break;   /* line inside the window: the path */
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * -TURN_SPEED);
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * TURN_SPEED);
    pulses_shown = turned;                   /* stays on screen after */
    key1_show(turned < TURN_WIN_MIN ? "K1 TURN SKIP" : "K1 TURN WINDOW");
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);

  /* The robot coasts a little after stopping, so the line can slip past the middle
   * sensors. Turning right moves the line to the robot's left, so turn back LEFT
   * (J1 forward, J3 back) until a middle sensor sees it again. */
  float return_side = -1.0f;                 /* overshot a right turn: line on the left */
  IR_Update(&ir);
  if (turned >= TURN_WIN_MIN && turned < TURN_WIN_MAX &&
      !ir.black[IR_LEFT_MIDDLE] && !ir.black[IR_RIGHT_MIDDLE]) {
    t = HAL_GetTick();
    while (HAL_GetTick() - t < SETTLE_MAX_MS) {
      IR_Update(&ir);
      if (ir.black[IR_LEFT_MIDDLE] || ir.black[IR_RIGHT_MIDDLE]) break;
      ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * SETTLE_SPEED);
      ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * -SETTLE_SPEED);
      key1_show("K1 TURN BACK");
    }
    ELCP_Motor_Stop(MOTOR_J1);
    ELCP_Motor_Stop(MOTOR_J3);
    HAL_Delay(200);
    return_side = 1.0f;                      /* if it coasted again, now to the right */
  }

  drive_to_goal("K1 RETURN", return_side);

  /* After the run: where the turn met lines, to set the window around the straight
   * path's count (and away from the junction branch's). */
  tft_prints(0, 0, "TURN LINES AT");
  unsigned shown = lines < TURN_LOG_MAX ? lines : TURN_LOG_MAX;
  for (unsigned i = 0; i < shown; i++)
    tft_prints(0, 1 + i, "LINE %u: %d", i + 1, (int)lines_at[i]);
  if (lines == 0) tft_prints(0, 1, "NONE");
  tft_prints(0, 8, "STOP AT %d", (int)turned);
  tft_prints(0, 9, "WIN %d-%d", (int)TURN_WIN_MIN, (int)TURN_WIN_MAX);
  tft_update(0);
  HAL_Delay(8000);
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
  MX_I2C1_Init();
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

  /* ToF: reset, scan I2C1, WHO_AM_I at every address that answers, start ranging.
   * Shows each device as "ADDR WHOAMI MMTT" (VL53L1X: 29 WHOAMI EACC). */
  boot_log("TOF INIT I2C1...");
  {
    ToF_Status st = ToF_Init();
    const ToF_Data *t = ToF_Get();
    /* boot_log keeps the pointer, so the lines live in static buffers (16 columns). */
    static char dev_line[2][17], tof_line[17];
    unsigned shown = t->found_count < 2 ? t->found_count : 2;
    if (t->found_count == 0) boot_log("I2C1 NO DEVICE");
    for (unsigned i = 0; i < shown; i++) {
      snprintf(dev_line[i], 17, "%02X WHOAMI %02X%02X", t->found[i].addr7,
               t->found[i].model_id, t->found[i].module_type);
      boot_log(dev_line[i]);
    }
    if (st == TOF_OK) snprintf(tof_line, sizeof tof_line, "TOF OK @0x%02X", t->addr7);
    else              snprintf(tof_line, sizeof tof_line, "TOF %s", ToF_StatusText(st));
    boot_log(tof_line);
  }

  HAL_Delay(1500);   /* keep the startup log on screen briefly */
  unsigned us_next = 0;
  uint32_t last_us = HAL_GetTick();
  uint32_t tof_retry = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Read sensors */
    IR_Update(&ir);
    ToF_Update();
    if (!ToF_IsRunning() && HAL_GetTick() - tof_retry >= 1000) {
      tof_retry = HAL_GetTick();             /* sensor down: restart once a second */
      ToF_Init();
    }
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
