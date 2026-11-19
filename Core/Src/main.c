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
 *             stop when US1 (front) < KEY1_STOP_MM (165 mm) twice in a row. Straight line, so
 *             steering is capped at LINE_TURN_RATIO of the speed (no junctions).
 * 3. PAUSE    1 s still.
 * 4. KEY OFF  turn right RIGHT_DEG (60 deg), then pull back a little.
 * 5. TURN     right on the spot; a line seen inside TURN_WIN_MIN..TURN_WIN_MAX pulses
 *             is the straight path back: stop and follow it. Lines outside are ignored.
 * 6. RETURN   same line PID and steering cap as step 2, until all 4 IR see black (T-line),
 *             then straight ahead 2:1 into the start zone, stop at GOAL_STOP_MM (110 mm).
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
#define KEY1_STOP_MM 165   /* key 1: stop this far from the key holder */
#define KEY2_STOP_MM 147   /* key 2: stop this far from the key holder */
#define GOAL_STOP_MM 110   /* return leg: after the T, drive straight 2:1 and stop this
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

/* Key 2 */
#define K2_BRANCH_N 2      /* turn right at this branch on the right (2nd = the D's line) */
#define BRANCH_LOCKOUT_MS 300 /* a branch counts once: next one only after this long
                               * without branch readings */
#define BRANCH_IGNORE_MS 500  /* ignore branches this soon after the start (start zone) */
#define BRANCH_MAX_MS 30000   /* safety: give up counting after this */
#define AXLE_FWD_PULSES 80 /* IR bar -> wheel axle, in J3 encoder pulses: drive this far
                            * past the junction before pivoting. CALIBRATE */
#define AXLE_FWD_MAX_MS 3000  /* safety for that move */
#define TURN90_WIN_MIN 100 /* 90 deg pivots at the junctions: the new line is the only one */
#define TURN90_WIN_MAX 320 /* in this window of pulses. CALIBRATE from the end screen */
#define K2_TURN_WIN_MIN 330 /* turn-around at the key 2 holder: the R's diagonal comes ~45 deg */
#define K2_TURN_WIN_MAX 480 /* before the vertical line, so MIN must be above the diagonal's
                             * count. CALIBRATE from the end screen's TURN line */

static float line_err;     /* -2 = line far left .. +2 = line far right; kept if the line is lost */
static float line_hist[4]; /* last 4 errors, so D measures the change over 20 ms */
static unsigned line_hi;
static float line_turn;
static uint32_t line_ms;
static bool line_keep_right; /* right-edge mode: at a split, take the right-hand line */

/* Fresh steering state, before each drive. */
static void line_reset(void) {
  line_err = line_turn = 0;
  line_hist[0] = line_hist[1] = line_hist[2] = line_hist[3] = 0;
  line_hi = 0;
  line_ms = 0;
  line_keep_right = false;
}

/* Reads the IR and updates line_err. Middle sensors first: the outer ones only count
 * when the line has left both middle ones, so a junction branch touching LL/RR can't
 * steer it. 3+ black (junction) reads as straight. Nothing seen: keep the last value.
 * Right-edge mode instead keeps the line's right edge between RM and RR (line under RM,
 * RR white), judged from the right side first: a second line on the left (a branch
 * splitting off left) is ignored while RM/RR still see the right one. */
static void line_read(void) {
  IR_Update(&ir);
  bool ll = ir.black[IR_LEFTMOST], lm = ir.black[IR_LEFT_MIDDLE];
  bool rm = ir.black[IR_RIGHT_MIDDLE], rr = ir.black[IR_RIGHTMOST];
  if (line_keep_right) {
    if (ll && lm && rm && rr) line_err = 0.0f;  /* T / crossing: straight */
    else if (rr && rm)  line_err = 1.0f;        /* edge drifted right: move right */
    else if (rr)        line_err = 2.0f;        /* far right */
    else if (rm)        line_err = 0.0f;        /* on target */
    else if (lm)        line_err = -1.0f;       /* line only left of centre: move left */
    else if (ll)        line_err = -2.0f;       /* far left */
    return;                                     /* nothing seen: keep the last value */
  }
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

/* "K<k> <what>" for the screen title. One shared buffer: use the result right away. */
static const char *ph(int k, const char *what) {
  static char b[17];
  snprintf(b, sizeof b, "K%d %s", k, what);
  return b;
}

/* Seeds the line follower with the side the line is on (-1 left, 0 ahead, +1 right),
 * used until a sensor sees the line. Same history, so no D kick at the start. */
static void line_start(float start_err) {
  line_reset();
  line_err = start_err;
  for (unsigned i = 0; i < 4; i++) line_hist[i] = start_err;
}

/* Follow the line to the wall ahead (key holder). Slow down inside APPROACH_MM, stop
 * when the front ultrasonic US1 reads under stop_mm twice in a row. Steering is capped
 * at LINE_TURN_RATIO of the speed, so a junction branch can't pull it off the straight. */
static void drive_to_wall(const char *phase, float start_err, int32_t stop_mm) {
  line_start(start_err);
  int close = 0;
  uint32_t last_us = 0;
  float speed = BASE_SPEED;
  while (close < 2) {
    line_drive(speed, LINE_TURN_RATIO * speed);
    if (HAL_GetTick() - last_us >= 60) {
      last_us = HAL_GetTick();
      int32_t d = Ultrasonic_ReadMm(0);   /* US1, front */
      us_mm[0] = d;
      close = (d >= 0 && d < stop_mm) ? close + 1 : 0;
      if (d >= 0 && d < APPROACH_MM) speed = APPROACH_SPEED;  /* slow final approach, stays slow */
    }
    key1_show(phase);
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
}

/* Follow the line (same capped steering) until all 4 IR see black on two fresh readings
 * in a row: a T-line. A T in the first ignore_ms is ignored. Motors keep running. */
static void follow_to_T(const char *phase, float start_err, uint32_t ignore_ms) {
  line_start(start_err);
  uint32_t t0 = HAL_GetTick(), seen_read = 0;
  int t_hits = 0;
  while (HAL_GetTick() - t0 < RETURN_MAX_MS) {
    line_drive(BASE_SPEED, LINE_TURN_RATIO * BASE_SPEED);
    if (line_ms != seen_read) {              /* a fresh IR reading (every LINE_DT_MS) */
      seen_read = line_ms;
      bool all = ir.black[IR_LEFTMOST] && ir.black[IR_LEFT_MIDDLE] &&
                 ir.black[IR_RIGHT_MIDDLE] && ir.black[IR_RIGHTMOST];
      t_hits = (all && HAL_GetTick() - t0 >= ignore_ms) ? t_hits + 1 : 0;
      if (t_hits >= 2) break;                /* T-line */
    }
    key1_show(phase);
  }
}

/* Straight ahead at the plain 2:1 ratio (no steering), slow inside APPROACH_MM, stop
 * when US1 reads under stop_mm twice in a row. GOAL_MAX_MS is a safety limit only. */
static void straight_to_wall(const char *phase, int32_t stop_mm) {
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
      close = (d >= 0 && d < stop_mm) ? close + 1 : 0;
      if (d >= 0 && d < APPROACH_MM) speed = APPROACH_SPEED;  /* slow final approach */
    }
    key1_show(phase);
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
}

/* Return to the start: follow the line to the T-line at the start zone, then straight
 * 2:1 into it and stop at GOAL_STOP_MM. */
static void drive_to_goal(int k, float start_err) {
  follow_to_T(ph(k, "RETURN"), start_err, GOAL_IGNORE_MS);
  straight_to_wall(ph(k, "GOAL"), GOAL_STOP_MM);
}

/* Lines met during a pivot, for calibrating its window (shown after the run). */
typedef struct {
  int32_t at[TURN_LOG_MAX];  /* pulses turned at each line met (middle white -> black) */
  unsigned n;                /* lines met (can exceed TURN_LOG_MAX) */
  int32_t stop;              /* pulses turned when the turn stopped */
} TurnLog;

/* Pivot on the spot (dir +1 = left, -1 = right) until a middle sensor sees a line inside
 * the window win_min..win_max encoder pulses of turning. Lines outside the window are
 * ignored and reaching win_max ends the turn, so it never swings on to another line.
 * need_leave: the sensors start on a line (at a junction), so it must be left first.
 * The robot coasts a little after stopping: if the line slipped past the middle
 * sensors it turns back slowly until one sees it again.
 * Returns the side the line is on if no sensor sees it (start_err for the next follow). */
static float pivot_to_line(float dir, int32_t win_min, int32_t win_max, bool need_leave,
                           TurnLog *log, const char *phase) {
  const ELCP_Motor_State_t *m1 = ELCP_Motor_GetState(MOTOR_J1);
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  int32_t p1 = m1->pulse_count, p3 = m3->pulse_count;
  int32_t turned = 0;
  bool was_middle = true;                    /* starts near a line: no edge yet */
  bool left_line = !need_leave;
  log->n = 0;
  uint32_t t = HAL_GetTick();
  while (HAL_GetTick() - t < TURN_MAX_MS && turned < win_max) {
    IR_Update(&ir);
    bool middle = ir.black[IR_LEFT_MIDDLE] || ir.black[IR_RIGHT_MIDDLE];
    turned = (int32_t)dir * ((m1->pulse_count - p1) - (m3->pulse_count - p3));
    if (middle && !was_middle) {             /* log every line met */
      if (log->n < TURN_LOG_MAX) log->at[log->n] = turned;
      log->n++;
    }
    was_middle = middle;
    if (!middle) left_line = true;
    if (middle && left_line && turned >= win_min) break;   /* line inside the window */
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * dir * TURN_SPEED);
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * -dir * TURN_SPEED);
    pulses_shown = turned;                   /* stays on screen after */
    key1_show(phase);
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);
  log->stop = turned;

  /* Turning moves the line the other way across the robot, so after coasting past it
   * the line is on the side we turned from: turn back that way, slowly. */
  float side = dir;                          /* right turn overshot: line on the left */
  IR_Update(&ir);
  if (turned >= win_min && turned < win_max &&
      !ir.black[IR_LEFT_MIDDLE] && !ir.black[IR_RIGHT_MIDDLE]) {
    t = HAL_GetTick();
    while (HAL_GetTick() - t < SETTLE_MAX_MS) {
      IR_Update(&ir);
      if (ir.black[IR_LEFT_MIDDLE] || ir.black[IR_RIGHT_MIDDLE]) break;
      ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * -dir * SETTLE_SPEED);
      ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * dir * SETTLE_SPEED);
      key1_show("TURN BACK");
    }
    ELCP_Motor_Stop(MOTOR_J1);
    ELCP_Motor_Stop(MOTOR_J3);
    HAL_Delay(200);
    side = -dir;                             /* if it coasted again, now the other side */
  }
  return side;
}

/* 1 s still after the button press. */
static void key_start(int k) {
  pulses_shown = -1;
  uint32_t t = HAL_GetTick();
  while (HAL_GetTick() - t < 1000) key1_show(ph(k, "START"));
}

/* At the key holder: stand still, turn right RIGHT_DEG, pull back a little. */
static void key_motion(int k) {
  uint32_t t = HAL_GetTick();
  while (HAL_GetTick() - t < TURN_PAUSE_MS) key1_show(ph(k, "PAUSE"));
  pulses_shown = 0;
  pivot_deg(-RIGHT_DEG, ph(k, "RIGHT"));
  t = HAL_GetTick();
  while (HAL_GetTick() - t < PULL_MS) {
    ELCP_Motor_SetSpeed(MOTOR_J1, J1_GAIN * -PULL_SPEED);
    ELCP_Motor_SetSpeed(MOTOR_J3, J3_GAIN * -PULL_SPEED);
    key1_show(ph(k, "PULL"));
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);
}

static void key1(void) {
  key_start(1);
  drive_to_wall("K1 TO KEY", 0.0f, KEY1_STOP_MM);
  key_motion(1);
  /* Turn around right: the straight path back is the only line inside the window. */
  TurnLog turn;
  float side = pivot_to_line(-1.0f, TURN_WIN_MIN, TURN_WIN_MAX, false, &turn, "K1 TURN");
  drive_to_goal(1, side);

  /* After the run: where the turn met lines, to set the window around the straight
   * path's count (and away from the junction branch's). */
  tft_prints(0, 0, "TURN LINES AT");
  unsigned shown = turn.n < TURN_LOG_MAX ? turn.n : TURN_LOG_MAX;
  for (unsigned i = 0; i < shown; i++)
    tft_prints(0, 1 + i, "LINE %u: %d", i + 1, (int)turn.at[i]);
  if (turn.n == 0) tft_prints(0, 1, "NONE");
  tft_prints(0, 8, "STOP AT %d", (int)turn.stop);
  tft_prints(0, 9, "WIN %d-%d", (int)TURN_WIN_MIN, (int)TURN_WIN_MAX);
  tft_update(0);
  HAL_Delay(8000);
}

/* ---- Key 2 -------------------------------------------------------------------------
 * Holder at the bottom of the D's vertical line. Route: east along the top line, right
 * at the 2nd branch on the right, down to the holder, key motion, turn around, up to
 * the top line (T), left, west to the start. Same follower, cap and stops as key 1.
 * ------------------------------------------------------------------------------------ */

/* Follow the line (capped steering) and count junctions, on two fresh readings in a row:
 *   t_junction false: a branch leaving to the RIGHT = rightmost sensor black while a
 *                     middle sensor is still on the line and the leftmost is not.
 *   t_junction true:  a T = BOTH outer sensors black with a middle one (a line out to
 *                     both sides; a branch on one side only, like the diagonal, never
 *                     shows this).
 * Each junction counts once; another can only be counted after BRANCH_LOCKOUT_MS without
 * junction readings. Junctions in the first ignore_ms are ignored. keep_right: follow
 * the line's right edge (a line splitting off left is ignored). Returns at junction n,
 * motors running. */
static void follow_to_junction(const char *title, unsigned n, bool t_junction,
                               float start_err, bool keep_right, uint32_t ignore_ms) {
  line_start(start_err);
  line_keep_right = keep_right;
  uint32_t t0 = HAL_GetTick(), seen_read = 0, last_branch = 0;
  unsigned count = 0;
  int hits = 0;
  bool in_branch = false;
  char shown[17];
  while (count < n && HAL_GetTick() - t0 < BRANCH_MAX_MS) {
    line_drive(BASE_SPEED, LINE_TURN_RATIO * BASE_SPEED);
    if (line_ms != seen_read) {              /* a fresh IR reading */
      seen_read = line_ms;
      uint32_t now = HAL_GetTick();
      bool mid = ir.black[IR_LEFT_MIDDLE] || ir.black[IR_RIGHT_MIDDLE];
      bool branch = t_junction
          ? (ir.black[IR_LEFTMOST] && ir.black[IR_RIGHTMOST] && mid)
          : (ir.black[IR_RIGHTMOST] && mid && !ir.black[IR_LEFTMOST]);
      branch = branch && now - t0 >= ignore_ms;
      hits = branch ? hits + 1 : 0;
      if (hits >= 2) {
        if (!in_branch) count++;             /* a new branch */
        in_branch = true;
        last_branch = now;
      } else if (in_branch && now - last_branch >= BRANCH_LOCKOUT_MS) {
        in_branch = false;                   /* branch passed: ready for the next */
      }
    }
    snprintf(shown, sizeof shown, "%s J%u", title, count);
    key1_show(shown);
  }
  line_keep_right = false;                   /* back to normal centring */
}

/* The IR bar is ahead of the wheels: keep following slowly for AXLE_FWD_PULSES (J3
 * encoder) so the wheels sit on the junction before pivoting. */
static void to_axle(const char *phase, int32_t pulses) {
  const ELCP_Motor_State_t *m3 = ELCP_Motor_GetState(MOTOR_J3);
  int32_t p3 = m3->pulse_count;
  uint32_t t = HAL_GetTick();
  while (m3->pulse_count - p3 < pulses && HAL_GetTick() - t < AXLE_FWD_MAX_MS) {
    line_drive(APPROACH_SPEED, LINE_TURN_RATIO * APPROACH_SPEED);
    key1_show(phase);
  }
  ELCP_Motor_Stop(MOTOR_J1);
  ELCP_Motor_Stop(MOTOR_J3);
  HAL_Delay(200);
}

/* Two screen rows for a pivot's log: "R90 STOP 180" and the lines met "  95 180". */
static void show_turn_log(unsigned row, const char *name, const TurnLog *l) {
  tft_prints(0, row, "%s STOP %d", name, (int)l->stop);
  char b[17] = " ";
  int len = 1;
  unsigned shown = l->n < TURN_LOG_MAX ? l->n : TURN_LOG_MAX;
  for (unsigned i = 0; i < shown && len < 12; i++)
    len += snprintf(b + len, sizeof b - len, " %d", (int)l->at[i]);
  tft_prints(0, row + 1, "%s", l->n ? b : "  NO LINES");
}

static void key2(void) {
  TurnLog r90 = {0}, turn = {0}, l90 = {0};
  key_start(2);
  follow_to_junction("K2 TOP", K2_BRANCH_N, false, 0.0f, false, BRANCH_IGNORE_MS);
                                             /* east, until the 2nd branch on the right */
  to_axle("K2 TO AXLE", AXLE_FWD_PULSES);
  float side = pivot_to_line(-1.0f, TURN90_WIN_MIN, TURN90_WIN_MAX, true, &r90, "K2 RIGHT 90");
  drive_to_wall("K2 TO KEY", side, KEY2_STOP_MM);
  key_motion(2);
  side = pivot_to_line(-1.0f, K2_TURN_WIN_MIN, K2_TURN_WIN_MAX, false, &turn, "K2 TURN");
  /* Up to the top line: the mirror of the right turn at the start. Right-edge following
   * (the diagonal splits off left), until the T = both outer sensors black. */
  follow_to_junction("K2 UP", 1, true, side, true, 0);
  to_axle("K2 TO AXLE", AXLE_FWD_PULSES);
  side = pivot_to_line(1.0f, TURN90_WIN_MIN, TURN90_WIN_MAX, true, &l90, "K2 LEFT 90");
  drive_to_goal(2, side);                    /* west to the start, like key 1 */

  /* After the run: each pivot's stop and the lines it met, to set the windows. */
  tft_prints(0, 0, "K2 DONE");
  show_turn_log(1, "R90", &r90);
  show_turn_log(3, "TURN", &turn);
  show_turn_log(5, "L90", &l90);
  tft_prints(0, 8, "90 WIN %d-%d", (int)TURN90_WIN_MIN, (int)TURN90_WIN_MAX);
  tft_prints(0, 9, "TRN WIN %d-%d", (int)K2_TURN_WIN_MIN, (int)K2_TURN_WIN_MAX);
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
    if (btn_read(BTN2)) key2();

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
