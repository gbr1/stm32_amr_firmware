/*
 * The MIT License
 *
 * Copyright (c) 2022 Giovanni di Dio Bruno https://gbr1.github.io
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "encoder.h"

Encoder::Encoder(timer_dev* Timer, unsigned char count_mode, uint8 pinA, uint8 pinB){
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  _TIMER = Timer;
	timer_init(_TIMER);

	(_TIMER->regs).gen->PSC = 1;

	(_TIMER->regs).gen->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1 | TIMER_CCMR1_CC2S_INPUT_TI2;

	(_TIMER->regs).gen->SMCR = count_mode;

	timer_resume(_TIMER);
}

void Encoder::reset(){
  timer_set_count(_TIMER, 0);
}

int16_t Encoder::getCount(){
  return timer_get_count(_TIMER);
}

unsigned char Encoder::getDirection(){
	return *bb_perip(&(_TIMER->regs).gen->CR1, TIMER_CR1_DIR_BIT);
}

void Encoder::invertPolarity(boolean p){
  if (p){
    (_TIMER->regs).gen->CCER |= (1<<TIMER_CCER_CC1P_BIT); //invert
  }
  else{
    (_TIMER->regs).gen->CCER &= ~( 1 << TIMER_CCER_CC1P_BIT);
  }
}
