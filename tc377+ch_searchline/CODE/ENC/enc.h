/*
 * enc.h
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#ifndef _ENC_H_
#define _ENC_H_

#include "zf_gpt12.h"
#include "zf_ccu6_pit.h"

#define ENC_GPT12 GPT12_T2
#define ENC_LSBPIN GPT12_T2INA_P00_7
#define ENC_DIRPIN GPT12_T2EUDA_P00_8

#define ENC_CCU6N CCU6_0
#define ENC_PITCH PIT_CH0
#define ENC_PITMS 5

void Enc_Init(void);
void Enc_Int(void);

#endif
