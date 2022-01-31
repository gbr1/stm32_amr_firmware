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
#ifndef __IMU_H__
#define __IMU_H__

#include <Wire.h>

#define MPU_ADDR 0x68
#define MPU_INIT 0x6B
#define MPU_RESET_AGT 0x68
#define MPU_RESET 0x6A
#define MPU_DATA 0x3B
#define MPU_ACC_SCALE 0x1C
#define MPU_GYRO_SCALE 0x1B
#define MPU_RAD 0.017453
#define MPU_EG 9.80665
#define MPU_DEVICE_RESET 0x80
#define MPU_WHOAMI 0x75


class Imu{
  private:
    TwoWire SWire;
    int16_t acx,acy,acz,tmp,gyx,gyy,gyz;
    float acc_ratio, gyro_ratio;
    float MPU_A_SCALE[4]={2.0,4.0,8.0,16.0};
    float MPU_G_SCALE[4]={250.0,500.0,1000.0,2000.0};
    uint8_t pin_sda,pin_scl;

  public:
    Imu(const uint8_t pin_sda_, const uint8_t pin_scl_):SWire(I2C_DEVICE){
      acc_ratio=MPU_EG*MPU_A_SCALE[0]/32768.0;
      gyro_ratio=MPU_RAD*MPU_G_SCALE[0]/32768.0;
      pin_sda=pin_sda_;
      pin_scl=pin_scl_;
    }

    ~Imu(){
      delete MPU_A_SCALE;
      delete MPU_G_SCALE;
    }

    bool isWorking(){
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_WHOAMI);
      SWire.endTransmission(false);
      SWire.requestFrom(MPU_ADDR,1);
      uint8_t code= SWire.read();
      if (code==0x68){
        return true;
      }
      return false;
    }

    void initialize(){
      pinMode(pin_sda,OUTPUT);
      pinMode(pin_scl,OUTPUT);
      digitalWrite(pin_sda,HIGH);
      digitalWrite(pin_scl,HIGH);
      delay(250);
      
      SWire.setClock(400000);
      SWire.begin();
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_INIT);
      SWire.write(1);
      SWire.endTransmission(true);
      delay(1000);
    }
    
    void updateData(){
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_DATA);
      SWire.endTransmission(false);
      SWire.requestFrom(MPU_ADDR,14,true);
      acx=SWire.read()<<8|SWire.read();
      acy=SWire.read()<<8|SWire.read();
      acz=SWire.read()<<8|SWire.read();
      tmp=SWire.read()<<8|SWire.read();
      gyx=SWire.read()<<8|SWire.read();
      gyy=SWire.read()<<8|SWire.read();
      gyz=SWire.read()<<8|SWire.read();
    }

    float getAccX(){ return acx*acc_ratio;}
    float getAccY(){ return acy*acc_ratio;}
    float getAccZ(){ return acz*acc_ratio;}
    float getTemp(){ return tmp/340.0+36.53;}
    float getGyroX(){ return gyx*gyro_ratio;}
    float getGyroY(){ return gyy*gyro_ratio;}
    float getGyroZ(){ return gyz*gyro_ratio;}

    float getGyroScale(){return gyro_ratio*32768.0/MPU_RAD;}
    float getAccScale(){return acc_ratio*32768.0/MPU_EG;}

    void setAccScale(uint8_t scale){
      if ((scale<0)&&(scale>3)) {return;}
      //acc scale (bits 4-3)
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_ACC_SCALE);
      SWire.write(scale<<3);
      SWire.endTransmission(true);
      acc_ratio=MPU_EG*MPU_A_SCALE[scale]/32768.0;
    }

    void setGyroScale(uint8_t scale){
      if ((scale<0)&&(scale>3)) {return;}
      //gyro scale (bits 4-3)
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_GYRO_SCALE);
      SWire.write(scale<<3);
      SWire.endTransmission(true);
      gyro_ratio=MPU_RAD*MPU_G_SCALE[scale]/32768.0;
    }

    void resetPaths(){
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_INIT);
      SWire.write(MPU_DEVICE_RESET);
      SWire.endTransmission(true);
      delay(100);
      SWire.beginTransmission(MPU_ADDR);
      SWire.write(MPU_RESET_AGT);
      SWire.write(7);
      SWire.endTransmission(true);
      delay(100);
    }
       
};


#endif
