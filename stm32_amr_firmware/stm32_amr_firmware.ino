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
 
#include "board_pins.h"
#include "motorcontroller.h"
#include "ucPack.h"
#include "imu.h"

ucPack packeter(100,65,35);

uint16_t t=0 ;

uint8_t c;
float f1,f2,f3,f4;

unsigned long timer_send = 0;
unsigned long timer_motor= 0;
unsigned long timer_imu= 0;
unsigned long timer_s= 0;

int cf=10;


MotorController motorA(MOTOR_A_PWM, MOTOR_A_IN2, MOTOR_A_IN1, MOTOR_A_TIM, COUNT_BOTH_CHANNELS,MOTOR_A_CH1,MOTOR_A_CH2, false, MOTOR_RATIO, float(cf));
MotorController motorB(MOTOR_B_PWM, MOTOR_B_IN2, MOTOR_B_IN1, MOTOR_B_TIM, COUNT_BOTH_CHANNELS,MOTOR_B_CH1,MOTOR_B_CH2, false, MOTOR_RATIO, float(cf));
MotorController motorC(MOTOR_C_PWM, MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2, true, MOTOR_RATIO, float(cf));
MotorController motorD(MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2, true, MOTOR_RATIO, float(cf));

float ref=0.0;

int state=0;

Imu mpu(I2C_SDA,I2C_SCL);

void setup() { 
  
  pinMode(LED_BUILTIN,OUTPUT);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  afio_remap(AFIO_REMAP_TIM2_PARTIAL_1);
  
  digitalWrite(LED_BUILTIN,HIGH);
  //pinMode(BATTERY_PIN, INPUT_ANALOG);

  serial_port.begin(115200);  
  
  motorA.init();
  motorB.init();
  motorC.init();
  motorD.init();

  //init mpu6050
  mpu.initialize();
  
  systick_attach_callback(tick); 
  
}


void loop() {
  /*
  if (timer_send>10){ 
    uint8_t dim = packeter.packetC4F('j',float(motorB.getRadAtS()),float(motorC.getRadAtS()),float(motorA.getRadAtS()),float(motorD.getRadAtS()));
    serial_port.write(packeter.msg,dim);
    timer_send=0;
  }
  */

  if (timer_send>=10){ 
    
    uint8_t dim = packeter.packetC4F('j',float(motorB.getRadAtS()),float(motorC.getRadAtS()),float(motorA.getRadAtS()),float(motorD.getRadAtS()));
    serial_port.write(packeter.msg,dim);
    
    /*
    serial_port.print(ref);
    serial_port.print(" ");
    serial_port.println(motorC.getRadAtS());
    */
    timer_send=0;
  }
/*
  if (timer_s>=5000){
    if (state==0){
      ref=0;
      systick_attach_callback(NULL);
      motorC.setReference(ref);
      systick_attach_callback(tick);
    }

    if (state==1){
      ref=3.8;
      systick_attach_callback(NULL);
      motorC.setReference(ref);
      systick_attach_callback(tick);
    }

    if (state==2){
      ref=-3.8;
      systick_attach_callback(NULL);
      motorC.setReference(ref);
      systick_attach_callback(tick);
    }

    if (state==3){
      ref=5.0;
      systick_attach_callback(NULL);
      motorC.setReference(ref);
      systick_attach_callback(tick);
    }

    if (state==4){
      ref=-5.0;
      systick_attach_callback(NULL);
      motorC.setReference(ref);
      systick_attach_callback(tick);
    }

    state++;
    if (state==5){
      state=0;
    }
    timer_s=0;
  }

  */

  if (timer_imu>=10){ 
    mpu.updateData(); 
    float ff=0.0;
    uint8_t dim = packeter.packetC8F('i',mpu.getAccY(),-mpu.getAccX(),mpu.getAccZ(),mpu.getGyroY(),-mpu.getGyroX(),mpu.getGyroZ(),mpu.getTemp(),ff);
    serial_port.write(packeter.msg,dim);
    timer_imu=0;
  }
  
  
  while(serial_port.available()>0){
    packeter.buffer.push(serial_port.read()); 
  }
  
  while(packeter.checkPayload()){
      packeter.unpacketC4F(c,f1,f2,f3,f4);
      systick_attach_callback(NULL);
      motorB.setReference(f1);
      motorC.setReference(f2);
      motorA.setReference(f3);
      motorD.setReference(f4);
      systick_attach_callback(tick);
  }
  
}


void updateMotors(){
  if (timer_motor>=cf){
    digitalWrite(LED_BUILTIN,LOW);
    //unsigned long t = micros();
    motorA.update();
    motorB.update();
    motorC.update();
    motorD.update();
    digitalWrite(LED_BUILTIN,HIGH);
    //serial_port.println(micros()-t);
    timer_motor=0;
  }
}




void tick(void){
  timer_motor++;
  timer_send++;
  timer_imu++;
  updateMotors();
}
