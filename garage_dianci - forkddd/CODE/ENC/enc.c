/*
 * enc.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "enc.h"

int16 enc_spd;

void Enc_Init(void)
{
    gpt12_init(ENC_GPT12,ENC_LSBPIN,ENC_DIRPIN);
    pit_interrupt_ms(ENC_CCU6N,ENC_PITCH,ENC_PITMS);
    pit_enable_interrupt(ENC_CCU6N,ENC_PITCH);
    pit_start(ENC_CCU6N,ENC_PITCH);
}

void Enc_Int(void)
{
    enc_spd=gpt12_get(ENC_GPT12);
    gpt12_clear(ENC_GPT12);
}
