#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------------- VL53L1X ToF (single sensor) ---------------- */
typedef struct {
    uint16_t address, distance_mm;
    uint8_t range_status;
    uint32_t signal_kcps, ambient_kcps; /* total rates, raw 9.7 MCPS multiplied by 8 */
    uint32_t last_ready_ms, sample_id; /* increments for each ready result, even invalid */
    bool initialized, valid;
} AR_ToF;
/* Startup can block for bounded boot/calibration waits; call only disarmed.
 * address is shifted 7-bit I2C address, usually 0x52. */
bool AR_ToFInit(AR_ToF *s, uint16_t address);
/* One data-ready check per call; no busy wait. 1 valid new sample,
 * 0 pending/invalid range, -1 communication/500ms ready timeout (reinit needed). */
int AR_ToFPoll(AR_ToF *s);

/* ---------------- Two ToF sensors on I2C1 PB6/PB7 (XSHUT PB5 / PB4) ---------------- */
bool AR_ToFBoardInit(void);
const char *AR_ToFBoardStatus(void);
/* Also recovers a failed sensor (power-cycle + re-address), at most one per call,
 * each sensor at most once per second. A recovery attempt can block ~1 s. */
void AR_ToFBoardPoll(void);
bool AR_ToFBoardSample(unsigned index, AR_ToF *sample);
int32_t AR_ToFBoardDistanceAt(unsigned index); /* 0=PB5 XSHUT, 1=PB4 XSHUT; -1 = no reading */
int32_t AR_ToFBoardDistance(void);

/* ---------------- Analog IR line sensors ----------------
 * Left to right:  PA3 (labelled Leftmiddle_IR) ADC1_IN3
 *                 PA2 (labelled Lefrtmost_IR) ADC3_IN2   - swapped on the robot
 *                 PC3 Middleright_IR ADC1_IN13
 *                 PC2 Rightmost_IR   ADC2_IN12
 * Readings are 12-bit ADC counts (0..4095). */
#define IR_DEFAULT_TRIGGER 1000u /* black/white threshold, ADC counts */
/* 0: reading below trigger = black. 1: reading at/above trigger = black. */
#define IR_BLACK_IS_HIGH 1

typedef enum {
    IR_LEFTMOST, IR_LEFT_MIDDLE, IR_RIGHT_MIDDLE, IR_RIGHTMOST, IR_COUNT
} IR_Index;

typedef struct {
    uint16_t trigger[IR_COUNT]; /* threshold per sensor */
    uint16_t raw[IR_COUNT];     /* last good ADC reading */
    bool black[IR_COUNT];       /* true = sensor is over the black line */
    bool ok[IR_COUNT];          /* false = last conversion failed */
} IR_Sensors;

/* Call after MX_ADC1/2/3_Init. */
void IR_Init(IR_Sensors *ir, uint16_t trigger);
void IR_Update(IR_Sensors *ir);

#endif
