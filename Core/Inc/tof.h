#ifndef TOF_H
#define TOF_H
/* VL53L1X time-of-flight sensor on I2C1 (SCL PB6, SDA PB7), XSHUT on PB4.
 *
 *   ToF_Init()    once at startup (blocking, up to ~0.7 s). Resets the sensor through
 *                 XSHUT, scans the bus, reads WHO_AM_I at every address that answers,
 *                 then configures the VL53L1X it found (long mode, 100 ms) and starts it.
 *   ToF_Update()  as often as you like (non-blocking). Collects a new measurement when
 *                 the sensor has one, about every 100 ms.
 *   ToF_DistanceMm()  latest distance, or -1 if there is no fresh valid reading.
 *
 * Datasheet DS12385: default address 0x52 (8-bit form, 0x29 as 7-bit), 16-bit register
 * index, multi-byte values MSB first, boot <= 1.2 ms after XSHUT rises, I2C <= 400 kHz,
 * AVDD/SCL/SDA/XSHUT absolute maximum 3.6 V. */
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define TOF_XSHUT_PORT        Xshut_GPIO_Port   /* PB4 */
#define TOF_XSHUT_PIN         Xshut_Pin
#define TOF_DEFAULT_ADDR7     0x29u             /* 0x52 >> 1 */
#define TOF_TIMING_BUDGET_MS  100u              /* long mode: 33/50/100/200/500 */
#define TOF_PERIOD_MS         100u              /* >= timing budget */
#define TOF_MAX_FOUND         4u                /* scan results kept */

typedef enum {
    TOF_OK = 0,
    TOF_ERR_BUS,        /* SDA/SCL held low, I2C1 could not be freed */
    TOF_ERR_NO_ACK,     /* nothing on the bus answered */
    TOF_ERR_WHO_AM_I,   /* devices answered, none identified as a VL53L1X */
    TOF_ERR_BOOT,       /* sensor never reported boot complete */
    TOF_ERR_I2C,        /* a transfer failed */
    TOF_ERR_TIMEOUT,    /* no new measurement for too long */
    TOF_NOT_STARTED,
} ToF_Status;

typedef struct {
    uint8_t addr7;      /* 7-bit address that answered */
    uint8_t model_id;   /* reg 0x010F, VL53L1X = 0xEA (0 if the read failed) */
    uint8_t module_type;/* reg 0x0110, VL53L1X = 0xCC */
} ToF_Found;

typedef struct {
    ToF_Status status;
    uint8_t addr7;            /* address in use, 0 = none */
    uint16_t distance_mm;     /* last measurement (even if not valid) */
    uint8_t range_status;     /* 0 = valid, see UM2510; 255 = unknown */
    uint32_t signal_kcps;     /* return signal rate per SPAD */
    uint32_t ambient_kcps;    /* ambient rate */
    bool valid;               /* last measurement was valid */
    uint32_t sample_ms;       /* HAL tick of the last valid measurement */
    uint32_t samples;         /* measurements collected (valid or not) */
    ToF_Found found[TOF_MAX_FOUND];   /* WHO_AM_I scan from the last ToF_Init */
    uint8_t found_count;      /* devices that answered (may exceed TOF_MAX_FOUND) */
} ToF_Data;

ToF_Status ToF_Init(void);
ToF_Status ToF_Update(void);
int32_t ToF_DistanceMm(void);          /* -1: no valid reading in the last 250 ms */
bool ToF_IsRunning(void);
const ToF_Data *ToF_Get(void);
const char *ToF_StatusText(ToF_Status s);   /* short text for the 16-column screen */

#endif
