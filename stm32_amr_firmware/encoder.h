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
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "Arduino.h"
#include <libmaple/timer.h>

#define COUNT_BOTH_CHANNELS		TIMER_SMCR_SMS_ENCODER3
#define COUNT_CH1				TIMER_SMCR_SMS_ENCODER1
#define COUNT_CH2				TIMER_SMCR_SMS_ENCODER2


class Encoder{
private:
  timer_dev * _TIMER;

public:
  Encoder();
  Encoder(timer_dev* , unsigned char , uint8 , uint8 );
  void reset();
  int16_t getCount();
  void setOffset(int16_t );
  unsigned char getDirection();
  void invertPolarity(boolean);
};


#endif
