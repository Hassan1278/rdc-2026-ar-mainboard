#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

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
