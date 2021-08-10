/*
 * enc.h
 *
 *  Created on: 2021?¨º5??9??
 *       
 */

#ifndef _ENC_H_
#define _ENC_H_

#include "zf_gpt12.h"
#include "zf_ccu6_pit.h"

#define ENC_GPT12 GPT12_T6
#define ENC_LSBPIN GPT12_T6INA_P20_3
#define ENC_DIRPIN GPT12_T6EUDA_P20_0

#define ENC_CCU6N CCU6_0
#define ENC_PITCH PIT_CH0
#define ENC_PITMS 5

void Enc_Init(void);
void Enc_Int(void);

#endif
