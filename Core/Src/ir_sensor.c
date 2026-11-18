#include "ir_sensor.h"
#include "adc.h"
#include "i2c.h"
#include "vl53l1_platform.h"
#include "vl53l1x_subset.h"
#include <string.h>

/* ======================= VL53L1X single sensor ======================= */
static int failed(AR_ToF *s) {
    s->initialized=false; s->valid=false; s->distance_mm=0; return -1;
}
bool AR_ToFInit(AR_ToF *s,uint16_t address) {
    if (!s) return false;
    memset(s,0,sizeof(*s)); s->range_status=255;
    if (address<0x10 || address>0xee || (address&1)) return false;
    s->address=address;
    VL53L1_PlatformBegin();
    uint32_t start=VL53L1_PlatformTick();
    uint8_t boot=0;
    while (!boot) {
        if (VL53L1X_BootState(address,&boot)) return false;
        if (boot) break;
        if ((uint32_t)(VL53L1_PlatformTick()-start)>=500) return false;
        VL53L1_PlatformDelay(2);
    }
    uint16_t id=0;
    if (VL53L1X_GetSensorId(address,&id) || id!=0xeacc ||
        VL53L1X_SensorInit(address) || VL53L1X_SetTimingBudgetInMs(address,50) ||
        VL53L1X_SetDistanceMode(address,2) ||
        VL53L1X_SetInterMeasurementInMs(address,100) ||
        VL53L1X_StartRanging(address)) return false;
    s->initialized=true; s->last_ready_ms=VL53L1_PlatformTick(); return true;
}
int AR_ToFPoll(AR_ToF *s) {
    if (!s) return -1;
    s->valid=false; s->distance_mm=0;
    if (!s->initialized) return -1;
    VL53L1_PlatformBegin();
    uint8_t ready=0;
    if (VL53L1X_CheckForDataReady(s->address,&ready)) return failed(s);
    if (!ready) {
        if ((uint32_t)(VL53L1_PlatformTick()-s->last_ready_ms)>=500) return failed(s);
        return 0;
    }
    uint16_t distance=0, signal=0, ambient=0;
    if (VL53L1X_GetRangeStatus(s->address,&s->range_status) ||
        VL53L1X_GetDistance(s->address,&distance) ||
        VL53L1_RdWord(s->address,VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0,&signal) ||
        VL53L1_RdWord(s->address,RESULT__AMBIENT_COUNT_RATE_MCPS_SD,&ambient) ||
        VL53L1X_ClearInterrupt(s->address)) return failed(s);
    s->last_ready_ms=VL53L1_PlatformTick(); s->sample_id++;
    s->signal_kcps=(uint32_t)signal*8u; s->ambient_kcps=(uint32_t)ambient*8u;
    s->valid=s->range_status==0 && distance!=0;
    if (s->valid) s->distance_mm=distance;
    return s->valid ? 1 : 0;
}

/* ======================= Two-sensor ToF board ======================= */
#define AR_TOF_FIRST_SENSOR 0
#define AR_TOF_SENSOR_COUNT 2
#define AR_TOF_RETRY_MS 1000u /* delay between recovery attempts for a sensor that is down */

static AR_ToF sensors[2], snapshots[2];
static const char *fault[2]={"TOF OFF","TOF OFF"}; /* why each sensor is down */
static uint32_t last_poll, sample_ms[2], last_attempt[2];
static int32_t distance[2] = {-1, -1};
static const uint16_t xshut[2]={AR_TOF_1_XSHUT_Pin,AR_TOF_2_XSHUT_Pin};
static const uint16_t addresses[2]={0x60,0x62}; /* 7-bit 0x30, 0x31 shifted for HAL */

const char *AR_ToFBoardStatus(void) {
    for (unsigned i=AR_TOF_FIRST_SENSOR;i<AR_TOF_FIRST_SENSOR+AR_TOF_SENSOR_COUNT;i++)
        if (!sensors[i].initialized) return fault[i];
    return "TOF READY";
}
/* HAL_I2C_Init software-resets I2C2, which also clears a latched BUSY state. */
static bool bus_init(void) { return HAL_I2C_Init(&hi2c2)==HAL_OK; }

/* Power-cycles sensor i through XSHUT so it boots at 0x52 whatever state it was in,
 * then initializes it and moves it to its own address. A sensor that is not running
 * is always held in shutdown, so 0x52 is free whenever this runs. */
static bool bring_up(unsigned i) {
    const bool first=i==0;
    sensors[i].initialized=false; sensors[i].valid=false;
    snapshots[i]=(AR_ToF){0}; distance[i]=-1;
    HAL_GPIO_WritePin(GPIOB,xshut[i],GPIO_PIN_RESET);
    HAL_Delay(2);
    fault[i]="I2C INIT FAIL";
    if (!bus_init()) return false;
    HAL_GPIO_WritePin(GPIOB,xshut[i],GPIO_PIN_SET);
    HAL_Delay(5);
    fault[i]=first?"T1 NO ACK":"T2 NO ACK";
    if (HAL_I2C_IsDeviceReady(&hi2c2,0x52,3,10)!=HAL_OK) goto off;
    fault[i]=first?"T1 INIT FAIL":"T2 INIT FAIL";
    if (!AR_ToFInit(&sensors[i],0x52)) goto off;
    /* VL53L1X I2C_SLAVE__DEVICE_ADDRESS: register takes the 7-bit address. */
    VL53L1_PlatformBegin();
    fault[i]=first?"T1 ADDR WRITE":"T2 ADDR WRITE";
    if (VL53L1_WrByte(0x52,0x0001,(uint8_t)(addresses[i]>>1))) goto off;
    sensors[i].address=addresses[i];
    uint8_t readback=0;
    fault[i]=first?"T1 ADDR CHECK":"T2 ADDR CHECK";
    if (VL53L1_RdByte(addresses[i],0x0001,&readback) || readback!=(addresses[i]>>1)) goto off;
    return true;
off:
    sensors[i].initialized=false; sensors[i].valid=false;
    HAL_GPIO_WritePin(GPIOB,xshut[i],GPIO_PIN_RESET);
    return false;
}

/* Call after MX_GPIO_Init and MX_I2C2_Init. */
bool AR_ToFBoardInit(void) {
    last_poll = 0;
    for (unsigned i=0;i<2;i++) {
        distance[i]=-1; sensors[i]=(AR_ToF){0}; snapshots[i]=(AR_ToF){0};
        sample_ms[i]=0; last_attempt[i]=0; fault[i]="TOF OFF";
    }
    /* Both sensors in shutdown (XSHUT is open drain: released = module pull-up). */
    HAL_GPIO_WritePin(GPIOB,AR_TOF_1_XSHUT_Pin|AR_TOF_2_XSHUT_Pin,GPIO_PIN_RESET);
    HAL_Delay(5);
    /* Add internal pull-ups on SCL/SDA in case the modules have none. */
    GPIO_InitTypeDef g={0};
    g.Pin=AR_TOF_SCL_Pin|AR_TOF_SDA_Pin; g.Mode=GPIO_MODE_AF_OD;
    g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB,&g);
    /* A sensor that fails here stays in shutdown and is retried from AR_ToFBoardPoll. */
    bool ready=true;
    for (unsigned i=AR_TOF_FIRST_SENSOR;i<AR_TOF_FIRST_SENSOR+AR_TOF_SENSOR_COUNT;i++) {
        last_attempt[i]=HAL_GetTick();
        if (!bring_up(i)) ready=false;
    }
    return ready;
}
void AR_ToFBoardPoll(void) {
    uint32_t now=HAL_GetTick();
    if(now-last_poll<20) return;
    last_poll=now;
    bool retried=false; /* at most one bring-up per call bounds the blocking time */
    for(unsigned i=AR_TOF_FIRST_SENSOR;i<AR_TOF_FIRST_SENSOR+AR_TOF_SENSOR_COUNT;i++) {
        if(!sensors[i].initialized) {
            distance[i]=-1;
            if(!retried && now-last_attempt[i]>=AR_TOF_RETRY_MS) {
                retried=true; last_attempt[i]=now; bring_up(i);
            }
            continue;
        }
        uint32_t before=sensors[i].sample_id;
        int result=AR_ToFPoll(&sensors[i]);
        if(sensors[i].sample_id!=before) snapshots[i]=sensors[i];
        if(result<0) {
            fault[i]=i==0?"T1 POLL FAIL":"T2 POLL FAIL";
            /* Dropped out while running: shut it down and retry on the next poll. */
            HAL_GPIO_WritePin(GPIOB,xshut[i],GPIO_PIN_RESET);
            last_attempt[i]=now-AR_TOF_RETRY_MS;
        }
        if(result==1) { distance[i]=sensors[i].distance_mm; sample_ms[i]=HAL_GetTick(); }
        else if(result<0 || sensors[i].range_status!=0 ||
                (sensors[i].sample_id!=before && !sensors[i].valid)) distance[i]=-1;
    }
}
int32_t AR_ToFBoardDistanceAt(unsigned index) {
    if(index<AR_TOF_FIRST_SENSOR || index>=AR_TOF_FIRST_SENSOR+AR_TOF_SENSOR_COUNT ||
       !sensors[index].initialized || HAL_GetTick()-sample_ms[index]>250) return -1;
    return distance[index];
}
int32_t AR_ToFBoardDistance(void) { return AR_ToFBoardDistanceAt(0); }

bool AR_ToFBoardSample(unsigned index, AR_ToF *sample) {
    if(!sample || index>=2 || !sensors[index].initialized || !snapshots[index].sample_id ||
       HAL_GetTick()-snapshots[index].last_ready_ms>250) return false;
    *sample=snapshots[index]; return true;
}

/* ======================= Analog IR line sensors ======================= */
static const struct { ADC_HandleTypeDef *adc; uint32_t channel; } ir_channels[IR_COUNT]={
    {&hadc3,ADC_CHANNEL_2},  /* PA2 Lefrtmost_IR   */
    {&hadc1,ADC_CHANNEL_3},  /* PA3 Leftmiddle_IR  */
    {&hadc1,ADC_CHANNEL_13}, /* PC3 Middleright_IR */
    {&hadc2,ADC_CHANNEL_12}, /* PC2 Rightmost_IR   */
};
/* ADC1 serves two sensors, so the channel is selected before every conversion. */
static bool ir_read(unsigned i,uint16_t *value) {
    ADC_HandleTypeDef *adc=ir_channels[i].adc;
    ADC_ChannelConfTypeDef c={0};
    c.Channel=ir_channels[i].channel; c.Rank=1;
    c.SamplingTime=ADC_SAMPLETIME_84CYCLES;
    if(HAL_ADC_ConfigChannel(adc,&c)!=HAL_OK || HAL_ADC_Start(adc)!=HAL_OK) return false;
    bool ok=HAL_ADC_PollForConversion(adc,2)==HAL_OK;
    if(ok) *value=(uint16_t)HAL_ADC_GetValue(adc);
    HAL_ADC_Stop(adc);
    return ok;
}
void IR_Init(IR_Sensors *ir,uint16_t trigger) {
    if(!ir) return;
    memset(ir,0,sizeof(*ir));
    for(unsigned i=0;i<IR_COUNT;i++) ir->trigger[i]=trigger;
    IR_Update(ir);
}
void IR_Update(IR_Sensors *ir) {
    if(!ir) return;
    for(unsigned i=0;i<IR_COUNT;i++) {
        uint16_t value;
        ir->ok[i]=ir_read(i,&value);
        if(!ir->ok[i]) continue;
        ir->raw[i]=value;
        bool high=value>=ir->trigger[i];
        ir->black[i]=IR_BLACK_IS_HIGH?high:!high;
    }
}
