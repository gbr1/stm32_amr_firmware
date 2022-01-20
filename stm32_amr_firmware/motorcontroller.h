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
#ifndef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__

#include "dcmotor.h"
#include "encoder.h"
#include "board_pins.h"
#include "motor_specs.h"

#define MEM_SIZE 5

class MotorController: public DCMotor, public Encoder{
private:
  float ratio;
  float reference;
  float error;
  float measure;
  float rad_factor;
  float actuation;
  float controller_freq;

  float kp=2.0;
  float ki=0.2;
  float kd=0.0;//0.1;
  float c_p;
  float c_i;
  float c_d;
  float p_error;

  float travel;

  float measure_memory[MEM_SIZE];
  int   id_memory;
  
  void addMemory(float _val);
  

  float checkLimit(float _val);

public:
  MotorController(uint8 _pwm, uint8 _in1, uint8 _in2,
                  timer_dev* _timenc, unsigned char _modec, uint8 _ch1, uint8 _ch2, bool _invert,
                  float _ratio, float _controller_freq);
  void setReference(float _val){reference=_val;}
  void setRadAtS(float _vel);
  float getRadAtS(){return measure;}
  void update();
  void clearMemory();
  float meanMemory();
  void init();

  float getTravel(){return travel;}
  void resetTravel(){travel=0.0;}

};

#endif
