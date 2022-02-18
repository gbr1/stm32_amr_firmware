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

// packeters to manage communication
ucPack packeter(100,65,35);

uint16_t t=0;


bool connected = false;

// code
uint8_t c;
// floats
float f1,f2,f3,f4;

unsigned long timer_send        = 0;
unsigned long timer_motor       = 0;
unsigned long timer_imu         = 0;
unsigned long timer_battery     = 0;
unsigned long timer_led         = 0;
unsigned long timer_timeout     = 0;

float battery=0.0;
uint8_t battery_cycle=0;

// timer in ms to update motors controllers
int cf=10;


MotorController motorA(MOTOR_A_PWM, MOTOR_A_IN2, MOTOR_A_IN1, MOTOR_A_TIM, COUNT_BOTH_CHANNELS,MOTOR_A_CH1,MOTOR_A_CH2, false, MOTOR_RATIO, float(cf));
MotorController motorB(MOTOR_B_PWM, MOTOR_B_IN2, MOTOR_B_IN1, MOTOR_B_TIM, COUNT_BOTH_CHANNELS,MOTOR_B_CH1,MOTOR_B_CH2, false, MOTOR_RATIO, float(cf));
MotorController motorC(MOTOR_C_PWM, MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2, true, MOTOR_RATIO, float(cf));
MotorController motorD(MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2, true, MOTOR_RATIO, float(cf));

float ref=0.0;

Imu mpu(I2C_SDA,I2C_SCL);

bool led=false;
uint16_t blink_time=1000;

float timeout=0.0;
bool checkTimeout = true;


void setup() { 
  
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  if (digitalRead(BUTTON_PIN)==LOW){
    serial_port.begin(115200);
    while(1){
      serial_port.println("PROGRAMMING MODE");
      for (int i=1; i<=100; i++){
        for(int j=0; j<100; j++){
          digitalWrite(LED_BUILTIN,LOW);
          delayMicroseconds(i);
          digitalWrite(LED_BUILTIN,HIGH);
          delayMicroseconds(100-i);
        }
      }
      for (int i=1; i<=100; i++){
        for(int j=0; j<100; j++){
          digitalWrite(LED_BUILTIN,LOW);
          delayMicroseconds(100-i);
          digitalWrite(LED_BUILTIN,HIGH);
          delayMicroseconds(i);
        }
      }
    }
  }




  
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  afio_remap(AFIO_REMAP_TIM2_PARTIAL_1);
  
  digitalWrite(LED_BUILTIN,HIGH);
  pinMode(BATTERY_PIN, INPUT_ANALOG);

  serial_port.begin(115200);  
  
  motorA.init();
  motorB.init();
  motorC.init();
  motorD.init();

  //init mpu6050
  mpu.initialize(); //this require 3s
  mpu.setAccScale(0);
  mpu.setGyroScale(0);
  
  systick_attach_callback(tick); 
  
  timer_timeout=millis();
  
}


void loop() {
  if (connected){
    //joint publisher
    if (timer_send>=10){ 
      uint8_t dim = packeter.packetC4F('j',float(motorB.getRadAtS()),float(motorC.getRadAtS()),float(motorA.getRadAtS()),float(motorD.getRadAtS()));
      serial_port.write(packeter.msg,dim);
      timer_send=0;
    }

    //imu publisher
    if (timer_imu>=10){ 
      mpu.updateData(); 
      float ff=0.0;
      uint8_t dim = packeter.packetC8F('i',-mpu.getAccY(),mpu.getAccX(),-mpu.getAccZ(),-mpu.getGyroY(),mpu.getGyroX(),-mpu.getGyroZ(),mpu.getTemp(),ff);
      serial_port.write(packeter.msg,dim);
      timer_imu=0;
    }

    // battery update
    if (timer_battery>=10){
      battery+=analogRead(BATTERY_PIN);
      battery_cycle++;
      timer_battery=0;
    }

    // battery publisher
    if (battery_cycle>=100){
      battery=battery*BATTERY_CONSTANT/100.0;
      uint8_t dim = packeter.packetC1F('b',battery);
      serial_port.write(packeter.msg,dim);
      battery=0.0;
      battery_cycle=0;
    }
  }
  
  if (((millis()-timer_timeout)>(timeout*1000))&&checkTimeout){
    connected=false;
    checkTimeout=false;
    blink_time=1000;
    systick_attach_callback(NULL);
    motorB.init();
    motorC.init();
    motorA.init();
    motorD.init();
    systick_attach_callback(tick); 
    mpu.setAccScale(0);
    mpu.setGyroScale(0);
    uint8_t d = packeter.packetC1F('s',0.0);
    serial_port.write(packeter.msg,d);
  }

  if (timer_led>blink_time){
    led=!led;
    digitalWrite(LED_BUILTIN,led);
    timer_led=0;
  }
  
  
  // check serial
  while(serial_port.available()>0){
    packeter.buffer.push(serial_port.read()); 
  }

  // check if any command was sent
  while(packeter.checkPayload()){
    c=packeter.payloadTop();
    if (!connected){
      if (c=='E'){
        packeter.unpacketC1F(c,timeout);
        uint8_t d = packeter.packetC1F('e',timeout);
        serial_port.write(packeter.msg,d);
        connected=true;
        blink_time=100;
        if (timeout<=0.0){
          checkTimeout=false;
        }
        else{
          checkTimeout=true;
        }
        timer_timeout=millis();
      }
    }
    else{
      //update time
      timer_timeout=millis();
      
      //joint control
      if (c=='J'){
        packeter.unpacketC4F(c,f1,f2,f3,f4);
        systick_attach_callback(NULL);
        motorB.setReference(f1);
        motorC.setReference(f2);
        motorA.setReference(f3);
        motorD.setReference(f4);
        systick_attach_callback(tick); 
        uint8_t d = packeter.packetC1F('x',0.0);
        serial_port.write(packeter.msg,d);
      }
      
      //stop the robot
      if (c=='S'){
        connected=false;
        blink_time=1000;
        systick_attach_callback(NULL);
        motorB.init();
        motorC.init();
        motorA.init();
        motorD.init();
        systick_attach_callback(tick); 
        mpu.setAccScale(0);
        mpu.setGyroScale(0);
        uint8_t d = packeter.packetC1F('s',0.0);
        serial_port.write(packeter.msg,d);
      }

      if (c=='G'){
        float acc_scale, gyro_scale;
        packeter.unpacketC2F(c,acc_scale,gyro_scale);
        if (acc_scale==2.0){
            mpu.setAccScale(0);
        }else{
          if (acc_scale==4.0){
            mpu.setAccScale(1);
          }else{
            if (acc_scale==8.0){
              mpu.setAccScale(2);
            }else{
              if (acc_scale==16.0){
                mpu.setAccScale(3);
              }else{
                acc_scale=-1.0;
              }
            }
          }
        }

        if (gyro_scale==250.0){
          mpu.setGyroScale(0);
        }else{
          if (gyro_scale==500.0){
            mpu.setGyroScale(1);
          }else{ 
            if (gyro_scale==1000.0){
              mpu.setGyroScale(2);
            }else{
              if (gyro_scale==2000.0){
                mpu.setGyroScale(3);
              }else{
                gyro_scale=-1.0;
              }
            }
          }
        }
        uint8_t d = packeter.packetC2F('g',acc_scale,gyro_scale);
        serial_port.write(packeter.msg,d);
      }
    }
  } 
}




// motors controllers update
void updateMotors(){
  if (timer_motor>=cf){
    motorA.update();
    motorB.update();
    motorC.update();
    motorD.update();
    timer_motor=0;
  }
}

// here timers are incremented and motors controllers are updated
void tick(void){
  timer_motor++;
  timer_send++;
  timer_imu++;
  timer_battery++;
  timer_led++;
  if (connected){
    updateMotors();
  }
}
