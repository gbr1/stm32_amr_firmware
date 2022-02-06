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
 
#ifndef __BOARD_PINS_H__
#define __BOARD_PINS_H__


#define LED_BUILTIN PC13
#define serial_port Serial1
#define USE_STM32_HW_SERIAL //is Serial1

#define BATTERY_PIN PA5
#define BATTERY_CONSTANT 0.008832117
#define BUTTON_PIN PC0

#define I2C_SDA PB10
#define I2C_SCL PB11
#define I2C_DEVICE 2

//motor A encoder is associated to TIM2; must call afio_remap(AFIO_REMAP_TIM2_FULL); & enableDebugPorts();
#define MOTOR_A_PWM PC9
#define MOTOR_A_IN1 PD2
#define MOTOR_A_IN2 PC12
#define MOTOR_A_CH1 PA15
#define MOTOR_A_CH2 PB3
#define MOTOR_A_TIM TIMER2

//motor B encoder is associated to TIM3; PB5 can go in conflict with debug interace, must call enableDebugPorts();
#define MOTOR_B_PWM PC8
#define MOTOR_B_IN1 PB4
#define MOTOR_B_IN2 PB5
#define MOTOR_B_CH1 PA6
#define MOTOR_B_CH2 PA7
#define MOTOR_B_TIM TIMER3

//motor C encoder is associated to TIM4
#define MOTOR_C_PWM PC7
#define MOTOR_C_IN1 PC5
#define MOTOR_C_IN2 PC4
#define MOTOR_C_CH1 PB6
#define MOTOR_C_CH2 PB7
#define MOTOR_C_TIM TIMER4

//motor D encoder is associated to TIM52
#define MOTOR_D_PWM PC6
#define MOTOR_D_IN1 PB0
#define MOTOR_D_IN2 PB1
#define MOTOR_D_CH1 PA0
#define MOTOR_D_CH2 PA1
#define MOTOR_D_TIM TIMER5


#endif
