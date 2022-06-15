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

// V1.2.1
 
#include "board_pins.h"
#include "motorcontroller.h"
#include "ucPack.h"
#include "imu.h"




// packeters to manage communication
ucPack packeter(210,65,35);
uint8_t dim=0;
float ff=0.0;


uint16_t t=0;

bool connected = false;

// code
uint8_t c;
// floats
float f1,f2,f3,f4;


volatile unsigned long timer_send        = 0;
volatile unsigned long timer_motor       = 0;
volatile unsigned long timer_imu         = 0;
volatile unsigned long timer_battery     = 0;
volatile unsigned long timer_led         = 0;
volatile unsigned long timer_timeout     = 0;
volatile uint16_t timer_cut = 0;

float battery=0.0;
uint8_t battery_cycle=0;

// timer in ms to update motors controllers
int cf=10;




Imu mpu(I2C_SDA,I2C_SCL);
float acc_scale, gyro_scale;

bool led=false;
uint16_t blink_time=1000;

float timeout=0.0;
bool checkTimeout = false;
bool joints_update = false;
bool ack_joints=false;
bool references_update=false;

MotorController motorA(MOTOR_A_PWM, MOTOR_A_IN2, MOTOR_A_IN1, MOTOR_A_TIM, COUNT_BOTH_CHANNELS,MOTOR_A_CH1,MOTOR_A_CH2, false, MOTOR_RATIO, float(cf));
MotorController motorB(MOTOR_B_PWM, MOTOR_B_IN2, MOTOR_B_IN1, MOTOR_B_TIM, COUNT_BOTH_CHANNELS,MOTOR_B_CH1,MOTOR_B_CH2, false, MOTOR_RATIO, float(cf));
MotorController motorC(MOTOR_C_PWM, MOTOR_C_IN1, MOTOR_C_IN2, MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2, true, MOTOR_RATIO, float(cf));
MotorController motorD(MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2, true, MOTOR_RATIO, float(cf));

void motor_update(void);
HardwareTimer timer(1);

bool motor_is_updating = false;
bool serial_used = false;


void setup() { 
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  pinMode(PA12,OUTPUT);
  pinMode(PA8,OUTPUT);
  pinMode(PA11,OUTPUT);
  pinMode(PB12,OUTPUT);
  pinMode(PB13,OUTPUT);
  pinMode(PB14,OUTPUT);

  digitalWrite(PA12,LOW);
  digitalWrite(PA8,LOW);
  digitalWrite(PA11,LOW);
  digitalWrite(PB12,LOW);
  digitalWrite(PB13,LOW);
  digitalWrite(PB14,LOW);

  debug_port.begin(115200);
  debug_port.println("debug");

  

  // trap to not enable afio remapping
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
  
  //init mpu6050
  mpu.initialize(); //this require 3s
  mpu.setAccScale(0);
  mpu.setGyroScale(0);
  
  motorA.start();
  motorB.start();
  motorC.start();
  motorD.start();

  //enable TIM1
  timer.pause();
  timer.setPeriod(10000); // in microseconds
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);      // overflow might be small
  timer.attachCompare1Interrupt(motor_update);
  timer.refresh();
  timer.resume();

  //motor warm up
  warm_up();

  timer.pause();
  motorB.force_stop();
  motorC.force_stop();
  motorA.force_stop();
  motorD.force_stop();

  systick_attach_callback(tick);
}



void loop() {
  if (connected){
    // check sync flag - 10ms timing by TIM1 inturrupt function
    if (joints_update){
        
      digitalWrite(PA12,HIGH);

      if (references_update){
        motorB.setReference(f1);
        motorC.setReference(f2);
        motorA.setReference(f3);
        motorD.setReference(f4);
        references_update=false;
      }
      
      motorA.update();
      motorB.update();
      motorC.update();
      motorD.update(); 

      digitalWrite(PA12,LOW);

      digitalWrite(PA8,HIGH);

      // joints publisher
      dim = packeter.packetC4F('j',motorB.getRadAtS(),motorC.getRadAtS(),motorA.getRadAtS(),motorD.getRadAtS());
      serial_port.write(packeter.msg,dim);
      joints_update=false;

      // imu publisher
      mpu.updateData(); 
      dim = packeter.packetC8F('i',-mpu.getAccY(),mpu.getAccX(),-mpu.getAccZ(),-mpu.getGyroY(),mpu.getGyroX(),-mpu.getGyroZ(),mpu.getTemp(),ff);
      serial_port.write(packeter.msg,dim);

      battery+=analogRead(BATTERY_PIN);
      battery_cycle++;

      if (battery_cycle>=100){
        battery=battery*BATTERY_CONSTANT/100.0;
        dim = packeter.packetC1F('b',battery);
        serial_port.write(packeter.msg,dim);
        battery=0.0;
        battery_cycle=0;
      }

      digitalWrite(PA8,LOW);

    }

  }
  

  // timeout is disabled, preserved if in the future is needed
  /*
  if ((timer_timeout>(timeout*1000))&&checkTimeout){
    connected=false;
    checkTimeout=false;
    timer.pause();
    blink_time=1000;
    motorB.init();
    motorC.init();
    motorA.init();
    motorD.init();
    mpu.setAccScale(0);
    mpu.setGyroScale(0);
    dim = packeter.packetC1F('s',ff);
    serial_port.write(packeter.msg,dim);
  }
  */

  // blink
  
  if (timer_led>blink_time){
    led=!led;
    digitalWrite(LED_BUILTIN,led);
    timer_led=0;
  }
  
  // data receiving blocks
  digitalWrite(PB13,HIGH);
  
  //debug_port.println("r");
  // check serial
  while(serial_port.available()>0){
    packeter.buffer.push(serial_port.read()); 
  }


  // check if any command was sent
  if(packeter.checkPayload()){
    c=packeter.payloadTop();
    timer_cut=0;

    if (!connected){
      if (c=='E'){
        packeter.unpacketC1F(c,timeout);
        dim = packeter.packetC1F('e',timeout);
        serial_port.write(packeter.msg,dim);
        connected=true;
        blink_time=100;
        /*
        if (timeout<=0.0){
          checkTimeout=false;
        }
        else{
          checkTimeout=true;
        }
        */
        
        motorB.init();
        motorC.init();
        motorA.init();
        motorD.init();

        timer.refresh();
        timer.resume();
        timer_cut=0;
        
        //timer_timeout=0;
      }
    }
    else{

      //update time
      //timer_timeout=0;
      
      //joint control
      if (c=='J'){
        digitalWrite(PB14,LOW);
        digitalWrite(PA11,HIGH);
        packeter.unpacketC4F(c,f1,f2,f3,f4);
        ack_joints=true;
        references_update=true;
        //timer.pause();
        /*
        debug_port.print(f1);
        debug_port.print(" ");
        debug_port.print(f2);
        debug_port.print(" ");
        debug_port.print(f3);
        debug_port.print(" ");
        debug_port.println(f4);
        */
        //timer.refresh();
        //timer.resume();  
        digitalWrite(PA11,LOW);
      }else

      
      //stop the robot
      if (c=='S'){
        stop_robot();
      } else


      if (c=='G'){
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
        dim = packeter.packetC2F('g',acc_scale,gyro_scale);
        serial_port.write(packeter.msg,dim);
      }
      else{
        digitalWrite(PB14,HIGH);
        //debug_port.println("C not detected");
      }

    }


  } 
  digitalWrite(PB13,LOW);

  // ack ros2 node that joints are updated
  if (ack_joints){
    dim = packeter.packetC1F('x',ff);
    serial_port.write(packeter.msg,dim);
    ack_joints=false;
  }

  if (connected && (timer_cut>500)){
    stop_robot();
  }
}


// here timers are incremented
void tick(void){
  timer_led++;
  timer_cut++;
  //timer_timeout++;
}

// interrupt function for TIM1 that enabled motors update at 100Hz
void motor_update(void){
  //digitalWrite(PA12,HIGH); 
  //sync flag for loop
  joints_update=true;
  //digitalWrite(PA12,LOW); 
}


void warm_up(){
  motorB.setReference(0.001);
  motorC.setReference(0.001);
  motorA.setReference(0.001);
  motorD.setReference(0.001);
  unsigned long t=0;
  t=millis();
  while(millis()-t<100){
    if (joints_update){
      motorA.update();
      motorB.update();
      motorC.update();
      motorD.update(); 
      joints_update=false;
    }
  }
  
  motorB.setReference(-0.001);
  motorC.setReference(-0.001);
  motorA.setReference(-0.001);
  motorD.setReference(-0.001);
  t=millis();
  while(millis()-t<100){
    if (joints_update){
      motorA.update();
      motorB.update();
      motorC.update();
      motorD.update(); 
      joints_update=false;
    }
  }
  
}


void stop_robot(){
  connected=false;
  blink_time=1000;
  timer.pause();
  motorB.init();
  motorC.init();
  motorA.init();
  motorD.init();
  mpu.setAccScale(0);
  mpu.setGyroScale(0);
  dim = packeter.packetC1F('s',ff);
  serial_port.write(packeter.msg,dim);
}