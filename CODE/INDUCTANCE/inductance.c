/*
*inductance.c
*/

#include "inductance.h"

#include "zf_ccu6_pit.h"


int16 induc_value[5];

void Induc_Init(void)
{
    adc_init(INDUC_ADC, INDUC_ECH1_PIN);
    adc_init(INDUC_ADC, INDUC_ECH2_PIN);
    adc_init(INDUC_ADC, INDUC_ECH3_PIN);
    adc_init(INDUC_ADC, INDUC_ECH4_PIN);
    adc_init(INDUC_ADC, INDUC_ECH5_PIN);

    pit_interrupt_ms(CCU6_0, PIT_CH1, 2);
    pit_enable_interrupt(CCU6_0, PIT_CH1);
    pit_start(CCU6_0, PIT_CH1);

    Induc_GetInfo();
}

void Induc_GetInfo(void)
{
    induc_value[0] = adc_mean_filter(INDUC_ADC, INDUC_ECH1_PIN, ADC_12BIT, INDUC_COUNT);
    induc_value[1] = adc_mean_filter(INDUC_ADC, INDUC_ECH2_PIN, ADC_12BIT, INDUC_COUNT);
    induc_value[2] = adc_mean_filter(INDUC_ADC, INDUC_ECH3_PIN, ADC_12BIT, INDUC_COUNT);
    induc_value[3] = adc_mean_filter(INDUC_ADC, INDUC_ECH4_PIN, ADC_12BIT, INDUC_COUNT);
    induc_value[4] = adc_mean_filter(INDUC_ADC, INDUC_ECH5_PIN, ADC_12BIT, INDUC_COUNT);
}