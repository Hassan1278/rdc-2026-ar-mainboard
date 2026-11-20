#include "ir_sensor.h"
#include "adc.h"
#include <string.h>

/* ======================= Analog IR line sensors ======================= */
static const struct { ADC_HandleTypeDef *adc; uint32_t channel; } ir_channels[IR_COUNT]={
    /* PA2/PA3 are swapped on the robot: PA3 is the leftmost sensor, PA2 left-middle. */
    {&hadc1,ADC_CHANNEL_3},  /* PA3 leftmost     */
    {&hadc3,ADC_CHANNEL_2},  /* PA2 left-middle  */
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
