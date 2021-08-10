

#ifndef _INDUCTANCE_H_
#define _INDUCTANCE_H_

#include "common.h"
#include "zf_vadc.h"

#define INDUC_ADC ADC_0
#define INDUC_ECH1_PIN ADC0_CH2_A2
#define INDUC_ECH2_PIN ADC0_CH3_A3
#define INDUC_ECH3_PIN ADC0_CH4_A4
#define INDUC_ECH4_PIN ADC0_CH5_A5
#define INDUC_ECH5_PIN ADC0_CH6_A6

#define INDUC_COUNT 10


void Induc_GetInfo(void);
void Induc_Init(void);

#endif