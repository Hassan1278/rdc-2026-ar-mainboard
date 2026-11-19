/* VL53L1X driver on I2C1 (SCL PB6, SDA PB7), XSHUT PB4. See tof.h.
 *
 * The register sequence (default configuration table, distance mode, timing budget,
 * inter-measurement period, result layout, range status codes) follows ST's VL53L1X
 * Ultra Lite Driver (UM2510). The datasheet (DS12385) does not publish the register
 * map, only the reference registers used for WHO_AM_I.
 *
 * Default configuration table: COPYRIGHT(c) 2018 STMicroelectronics.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE. */
#include "tof.h"
#include "i2c.h"
#include <string.h>

/* ---------------- Registers ---------------- */
#define REG_VHV_TIMEOUT_LOOP_BOUND   0x0008
#define REG_GPIO_HV_MUX_CTRL         0x0030  /* bit 4: interrupt polarity */
#define REG_GPIO_TIO_HV_STATUS       0x0031  /* bit 0: new data (vs polarity) */
#define REG_PHASECAL_TIMEOUT         0x004B  /* 0x14 short, 0x0A long */
#define REG_RANGE_TIMEOUT_A          0x005E  /* 16-bit */
#define REG_RANGE_VCSEL_PERIOD_A     0x0060
#define REG_RANGE_TIMEOUT_B          0x0061  /* 16-bit */
#define REG_RANGE_VCSEL_PERIOD_B     0x0063
#define REG_RANGE_VALID_PHASE_HIGH   0x0069
#define REG_INTERMEASUREMENT_PERIOD  0x006C  /* 32-bit */
#define REG_SD_WOI_SD0               0x0078  /* 16-bit */
#define REG_SD_INITIAL_PHASE_SD0     0x007A  /* 16-bit */
#define REG_INTERRUPT_CLEAR          0x0086
#define REG_MODE_START               0x0087  /* 0x40 start, 0x00 stop */
#define REG_RESULT                   0x0089  /* 17-byte result block */
#define REG_OSC_CALIBRATE_VAL        0x00DE  /* 16-bit */
#define REG_FIRMWARE_SYSTEM_STATUS   0x00E5  /* nonzero once booted */
#define REG_MODEL_ID                 0x010F  /* WHO_AM_I: 0xEA */
#define REG_MODULE_TYPE              0x0110  /* WHO_AM_I: 0xCC */

#define VL53L1X_MODEL_ID    0xEA
#define VL53L1X_MODULE_TYPE 0xCC

#define IO_TIMEOUT_MS  10u
#define SCL_PIN        GPIO_PIN_6
#define SDA_PIN        GPIO_PIN_7

/* Default configuration for registers 0x2D..0x87 (ULD VL51L1X_DEFAULT_CONFIGURATION).
 * Written in one auto-incrementing write (datasheet Figure 15). */
static const uint8_t default_config[] = {
    /* 0x2D */ 0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08,
    /* 0x35 */ 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    /* 0x3D */ 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    /* 0x45 */ 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    /* 0x4D */ 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    /* 0x55 */ 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    /* 0x5D */ 0x00, 0x01, 0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    /* 0x65 */ 0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    /* 0x6D */ 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 0x75 */ 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    /* 0x7D */ 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    /* 0x85 */ 0x01, 0x00, 0x00,
};
_Static_assert(sizeof default_config == 0x87 - 0x2D + 1, "config covers 0x2D..0x87");

/* ---------------- State ---------------- */
static ToF_Data d = { .status = TOF_NOT_STARTED, .range_status = 255 };
static uint16_t dev8;            /* 8-bit address used by HAL (7-bit << 1) */
static uint8_t int_polarity;     /* data ready when TIO_HV_STATUS bit 0 == this */
static uint16_t valid_mm;
static uint32_t last_ready_ms, last_check_ms;

/* ---------------- I2C register access (16-bit index, MSB first) ---------------- */
static bool rd(uint16_t reg, uint8_t *buf, uint16_t n) {
    return HAL_I2C_Mem_Read(&hi2c1, dev8, reg, I2C_MEMADD_SIZE_16BIT, buf, n,
                            IO_TIMEOUT_MS) == HAL_OK;
}
static bool wr(uint16_t reg, const uint8_t *buf, uint16_t n) {
    return HAL_I2C_Mem_Write(&hi2c1, dev8, reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *)buf, n,
                             IO_TIMEOUT_MS) == HAL_OK;
}
static bool rd8(uint16_t reg, uint8_t *v) { return rd(reg, v, 1); }
static bool rd16(uint16_t reg, uint16_t *v) {
    uint8_t b[2];
    if (!rd(reg, b, 2)) return false;
    *v = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}
static bool wr8(uint16_t reg, uint8_t v) { return wr(reg, &v, 1); }
static bool wr16(uint16_t reg, uint16_t v) {
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    return wr(reg, b, 2);
}
static bool wr32(uint16_t reg, uint32_t v) {
    uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    return wr(reg, b, 4);
}

/* ---------------- Helpers ---------------- */
static void delay_us(uint32_t us) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT, n = us * (SystemCoreClock / 1000000u);
    while (DWT->CYCCNT - start < n) {}
}

static ToF_Status fail(ToF_Status s) {
    d.status = s;
    d.valid = false;
    return s;
}

/* Frees a stuck bus (a device left mid-byte holding SDA low) with 9 SCL pulses and a
 * STOP, then re-initialises I2C1, which also clears a stuck BUSY flag. Internal
 * pull-ups stay on afterwards in case the module has none. */
static bool bus_recover(void) {
    HAL_I2C_DeInit(&hi2c1);
    GPIO_InitTypeDef g = {0};
    g.Pin = SCL_PIN | SDA_PIN; g.Mode = GPIO_MODE_OUTPUT_OD; g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, SCL_PIN | SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &g);
    delay_us(20);
    for (int i = 0; i < 9 && !HAL_GPIO_ReadPin(GPIOB, SDA_PIN); i++) {
        HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_RESET); delay_us(5);
        HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_SET);   delay_us(5);
    }
    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_RESET); delay_us(5);   /* STOP */
    HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_SET);   delay_us(5);
    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_SET);   delay_us(5);
    bool free = HAL_GPIO_ReadPin(GPIOB, SCL_PIN) && HAL_GPIO_ReadPin(GPIOB, SDA_PIN);

    HAL_I2C_Init(&hi2c1);                    /* MspInit puts PB6/PB7 back on AF4 */
    g.Mode = GPIO_MODE_AF_OD; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    return free;
}

/* Scans first..last and reads WHO_AM_I (model ID, module type) at each address that
 * answers. Returns the address of the VL53L1X, preferring 0x29, or 0 if none. */
static uint8_t scan_who_am_i(uint8_t first, uint8_t last) {
    uint8_t vl = 0;
    d.found_count = 0;
    for (unsigned a = first; a <= last; a++) {
        HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 2);
        if (r == HAL_BUSY) bus_recover();    /* stuck BUSY flag: free it, keep going */
        if (r != HAL_OK) continue;
        uint8_t model = 0, type = 0;
        dev8 = (uint16_t)(a << 1);
        if (!rd8(REG_MODEL_ID, &model)) model = 0;
        if (!rd8(REG_MODULE_TYPE, &type)) type = 0;
        if (d.found_count < TOF_MAX_FOUND)
            d.found[d.found_count] = (ToF_Found){ a, model, type };
        d.found_count++;
        bool is_vl = model == VL53L1X_MODEL_ID && type == VL53L1X_MODULE_TYPE;
        if (is_vl && (!vl || a == TOF_DEFAULT_ADDR7)) vl = a;
    }
    return vl;
}

static bool data_ready(bool *ready) {
    uint8_t v;
    if (!rd8(REG_GPIO_TIO_HV_STATUS, &v)) return false;
    *ready = (v & 0x01) == int_polarity;
    return true;
}

static bool timing_budget_regs(uint16_t ms, uint16_t *a, uint16_t *b) {  /* long mode */
    switch (ms) {
    case 33:  *a = 0x0060; *b = 0x006E; return true;
    case 50:  *a = 0x00AD; *b = 0x00C6; return true;
    case 100: *a = 0x01CC; *b = 0x01EA; return true;
    case 200: *a = 0x02D9; *b = 0x02F8; return true;
    case 500: *a = 0x048F; *b = 0x04A4; return true;
    default:  return false;
    }
}

/* ULD range status codes: 0 valid, 1 sigma fail, 2 signal fail, 4 phase out of bounds,
 * 7 wrap-around, 255 unknown, ... */
static uint8_t map_range_status(uint8_t raw) {
    switch (raw & 0x1F) {
    case 9:  return 0;   case 6:  return 1;   case 4:  return 2;   case 8:  return 3;
    case 5:  return 4;   case 3:  return 5;   case 19: return 6;   case 7:  return 7;
    case 12: return 9;   case 18: return 10;  case 22: return 11;  case 23: return 12;
    case 13: return 13;  default: return 255;
    }
}

/* Sensor off, free the bus while it can't interfere, sensor on and booted.
 * XSHUT is open drain with pull-up, so "high" is the pull-up (3.3 V, within the
 * 3.6 V absolute maximum) and low holds the sensor in hardware standby. */
static bool reset_sensor(void) {
    GPIO_InitTypeDef x = {0};
    x.Pin = TOF_XSHUT_PIN; x.Mode = GPIO_MODE_OUTPUT_OD; x.Pull = GPIO_PULLUP;
    x.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TOF_XSHUT_PORT, &x);
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    bool bus_ok = bus_recover();
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(3);                            /* boot takes up to 1.2 ms */
    return bus_ok;
}

/* ---------------- Public ---------------- */
ToF_Status ToF_Init(void) {
    memset(&d, 0, sizeof d);
    d.range_status = 255;
    d.status = TOF_NOT_STARTED;
    valid_mm = 0;

    /* 1. Reset the sensor through XSHUT and free the bus. */
    if (!reset_sensor()) return fail(TOF_ERR_BUS);

    /* 2. Scan + WHO_AM_I. Retry for 50 ms in case the sensor is slow to appear. */
    uint8_t addr = 0;
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        addr = scan_who_am_i(0x08, 0x77);
        if (addr || HAL_GetTick() - t0 >= 50) break;
        HAL_Delay(5);
    }
    if (d.found_count == 0) return fail(TOF_ERR_NO_ACK);
    if (!addr) return fail(TOF_ERR_WHO_AM_I);
    d.addr7 = addr;
    dev8 = (uint16_t)(addr << 1);

    /* 3. Wait for the firmware to finish booting. */
    t0 = HAL_GetTick();
    for (;;) {
        uint8_t booted = 0;
        if (rd8(REG_FIRMWARE_SYSTEM_STATUS, &booted) && booted) break;
        if (HAL_GetTick() - t0 >= 500) return fail(TOF_ERR_BOOT);
        HAL_Delay(2);
    }

    /* 4. Load the default configuration and run one measurement so the sensor
     *    calibrates (VHV), as the ULD SensorInit does. */
    uint8_t mux;
    if (!wr(0x002D, default_config, sizeof default_config) ||
        !rd8(REG_GPIO_HV_MUX_CTRL, &mux)) return fail(TOF_ERR_I2C);
    int_polarity = (mux & 0x10) ? 0 : 1;
    if (!wr8(REG_INTERRUPT_CLEAR, 0x01) || !wr8(REG_MODE_START, 0x40))
        return fail(TOF_ERR_I2C);
    t0 = HAL_GetTick();
    for (bool ready = false; !ready; ) {
        if (!data_ready(&ready)) return fail(TOF_ERR_I2C);
        if (!ready && HAL_GetTick() - t0 >= 1000) return fail(TOF_ERR_TIMEOUT);
        if (!ready) HAL_Delay(2);
    }
    if (!wr8(REG_INTERRUPT_CLEAR, 0x01) || !wr8(REG_MODE_START, 0x00) ||
        !wr8(REG_VHV_TIMEOUT_LOOP_BOUND, 0x09) || !wr8(0x000B, 0x00))
        return fail(TOF_ERR_I2C);

    /* 5. Long distance mode (up to ~4 m in the dark). */
    if (!wr8(REG_PHASECAL_TIMEOUT, 0x0A) || !wr8(REG_RANGE_VCSEL_PERIOD_A, 0x0F) ||
        !wr8(REG_RANGE_VCSEL_PERIOD_B, 0x0D) || !wr8(REG_RANGE_VALID_PHASE_HIGH, 0xB8) ||
        !wr16(REG_SD_WOI_SD0, 0x0F0D) || !wr16(REG_SD_INITIAL_PHASE_SD0, 0x0E0E))
        return fail(TOF_ERR_I2C);

    /* 6. Timing budget and inter-measurement period. */
    uint16_t ta, tb, osc;
    if (!timing_budget_regs(TOF_TIMING_BUDGET_MS, &ta, &tb)) return fail(TOF_ERR_I2C);
    if (!wr16(REG_RANGE_TIMEOUT_A, ta) || !wr16(REG_RANGE_TIMEOUT_B, tb) ||
        !rd16(REG_OSC_CALIBRATE_VAL, &osc)) return fail(TOF_ERR_I2C);
    uint32_t period = (uint32_t)(osc & 0x3FF) * TOF_PERIOD_MS * 1075u / 1000u;
    if (!wr32(REG_INTERMEASUREMENT_PERIOD, period)) return fail(TOF_ERR_I2C);

    /* 7. Start continuous ranging. */
    if (!wr8(REG_INTERRUPT_CLEAR, 0x01) || !wr8(REG_MODE_START, 0x40))
        return fail(TOF_ERR_I2C);
    last_ready_ms = last_check_ms = HAL_GetTick();
    d.status = TOF_OK;
    return TOF_OK;
}

ToF_Status ToF_Update(void) {
    if (d.status != TOF_OK) return d.status;
    uint32_t now = HAL_GetTick();
    if (now - last_check_ms < 10) return TOF_OK;   /* keep bus traffic light */
    last_check_ms = now;

    bool ready;
    if (!data_ready(&ready)) return fail(TOF_ERR_I2C);
    if (!ready) {
        if (now - last_ready_ms > 3u * TOF_PERIOD_MS + 200u) return fail(TOF_ERR_TIMEOUT);
        return TOF_OK;
    }

    /* Result block from 0x0089: [0] range status, [7..8] ambient, [13..14] distance,
     * [15..16] signal per SPAD. Then clear the interrupt so the next result can come. */
    uint8_t r[17];
    if (!rd(REG_RESULT, r, sizeof r) || !wr8(REG_INTERRUPT_CLEAR, 0x01))
        return fail(TOF_ERR_I2C);
    last_ready_ms = now;
    d.samples++;
    d.range_status = map_range_status(r[0]);
    d.distance_mm  = (uint16_t)((r[13] << 8) | r[14]);
    d.ambient_kcps = (uint32_t)((r[7] << 8) | r[8]) * 8u;
    d.signal_kcps  = (uint32_t)((r[15] << 8) | r[16]) * 8u;
    d.valid = d.range_status == 0 && d.distance_mm != 0;
    if (d.valid) { valid_mm = d.distance_mm; d.sample_ms = now; }
    return TOF_OK;
}

int32_t ToF_DistanceMm(void) {
    if (d.status != TOF_OK || d.sample_ms == 0 || HAL_GetTick() - d.sample_ms > 250)
        return -1;
    return valid_mm;
}

bool ToF_IsRunning(void) { return d.status == TOF_OK; }

const ToF_Data *ToF_Get(void) { return &d; }

const char *ToF_StatusText(ToF_Status s) {
    switch (s) {
    case TOF_OK:           return "OK";
    case TOF_ERR_BUS:      return "BUS HELD LOW";
    case TOF_ERR_NO_ACK:   return "NO ACK";
    case TOF_ERR_WHO_AM_I: return "NOT VL53L1X";
    case TOF_ERR_BOOT:     return "NO BOOT";
    case TOF_ERR_I2C:      return "I2C ERROR";
    case TOF_ERR_TIMEOUT:  return "NO DATA";
    default:               return "OFF";
    }
}
