////////////////////////////////////////////////////////////////////////////
//
//  This file is part of MPU9150Lib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

union float2bytes {
  float f;
  uint8_t b[4];
};
float2bytes f2b;

/* ****************************************************************
   ***                        ARDUINO                           ***
   ****************************************************************/
// Debug flag
boolean debug = false;

// Frame format:       {ST, ADD,    quaternions,      accel xyz,   gyro xyz,   slider, thumb joy, trackball,        ...         SP }
uint8_t transmit[36] = {60, 191, 0,0, 0,0, 0,0, 0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,  0,0,    0,0,0,0,   0,0,0,0,   0x00, 0x00, 0x00, 90 };


/* ****************************************************************
   ***                        SLIDER                            ***
   ****************************************************************/
boolean slider = false;
int sliderPin = 2; // Analog read A2 pin
int sliderVal = 0;
int sliderValMin = 0;
int sliderValMax = 1023;
//float sliderValF = 0.0;
int sliderTransmitPos = 22; // First position of slider values in the transmit array
int sliderVals = 2; // Number of positions occupied by slider in the transmit array


/* ****************************************************************
   ***                    THUMB JOYSTICK                        ***
   ****************************************************************/
// thumbJoy pinout (L to R):
// Xout - 5V - Yout - gnd
boolean thumbJoy = false;
int thumbPinX = 0; // Analog read A0 pin
int thumbPinY = 1; // Analog read A1 pin
int thumbValX = 0;
int thumbValY = 0;
int thumbValXMin = 224;
int thumbValXMax = 890;
int thumbValYMin = 20;
int thumbValYMax = 870;
//float thumbValXF = 0.0;
//float thumbValYF = 0.0;
int thumbTransmitPos = 24; // First position of the thumb joystick values in the transmit array
int thumbVals = 4; // Number of positions occupied by thumbJoy in the transmit array


/* ****************************************************************
   ***                       TRACKBALL                          ***
   ****************************************************************/
boolean trackball = false;
int tbLedBluePin = 11; // Digital out 8
int tbLedRedPin = 10; // Digital out 9
int tbLedGreePinn = 9; // Digital out 10
int tbLedWhitePin = 8; // Digital out 11
int tbWheelUpPin = 3; // Digital in 3 (PCInt)
int tbWheelDownPin = 4; // Digital in 4 (PCInt)
int tbWheelLeftPin = 5; // Digital in 5 (PCInt)
int tbWheelRightPin = 6; // Digital in 6 (PCInt)
int tbButtonPin = 7; // Digital in 7 (PCInt)

int tbTransmitPos = 28; // First position of the trackball values in the transmit array
int tbVals = 4; // Number of positions occupied by trackball in the transmit array

volatile int tbWheelVertCntRaw = 120;
volatile int tbWheelHorizCntRaw = 120;
int tbWheelCntMax = 240;
int tbWheelCntMin = 0;
int tbButtonPressed = 0;


/* ****************************************************************
   ***                        MPU-9150                          ***
   ****************************************************************/

MPU9150Lib MPU;                                            // the MPU object

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69
#define  DEVICE_TO_USE    0

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE   40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.print("Arduino9150 starting using device "); Serial.println(DEVICE_TO_USE);
  Wire.begin();
  
  // Slider setuo
  if(slider) {
    if(debug) Serial.println("Slider enabled!");
  }
  else {
    for(int i = 0; i < sliderVals; i++) {
      transmit[sliderTransmitPos + i] = 100 + i;
    }
  }
  
  // Thumb joystick setup
  if(thumbJoy) {
    if(debug) Serial.println("Thumb joystick enabled!");
  }
  else {
    for(int i = 0; i < thumbVals; i++) {
      transmit[thumbTransmitPos + i] = 120 + i;
    }
  }
  
  // Trackball setup
  if(trackball) {
    if(debug) Serial.println("Trackball enabled!");
    pinMode(tbWheelUpPin, INPUT);
    pinMode(tbWheelDownPin, INPUT);
    pinMode(tbWheelLeftPin, INPUT);
    pinMode(tbWheelRightPin, INPUT);
    pinMode(tbButtonPin, INPUT_PULLUP);
    
//    attachPinChangeInterrupt(tbButtonPin, tbButtonInt, CHANGE);
//    attachPinChangeInterrupt(tbWheelUpPin, tbWheelUpInt, RISING);
//    attachPinChangeInterrupt(tbWheelDownPin, tbWheelDownInt, RISING);
//    attachPinChangeInterrupt(tbWheelLeftPin, tbWheelLeftInt, RISING);
//    attachPinChangeInterrupt(tbWheelRightPin, tbWheelRightInt, RISING);
  }
  else {
    for(int i = 0; i < tbVals; i++) {
      transmit[tbTransmitPos + i] = 140 + i;
    }
  }

  // MPU initialization
  MPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{  
  MPU.selectDevice(DEVICE_TO_USE);                         // only needed if device has changed since init but good form anyway
  if (MPU.read()) {                                        // get the latest data if ready yet
//    MPU.printQuaternion(MPU.m_rawQuaternion);              // print the raw quaternion from the dmp
//    MPU.printVector(MPU.m_rawMag);                         // print the raw mag data
//    MPU.printVector(MPU.m_rawAccel);                       // print the raw accel data
//    MPU.printAngles(MPU.m_dmpEulerPose);                   // the Euler angles from the dmp quaternion
//    MPU.printVector(MPU.m_calAccel);                       // print the calibrated accel data
//    MPU.printVector(MPU.m_calMag);                         // print the calibrated mag data
//    MPU.printAngles(MPU.m_fusedEulerPose);                 // print the output of the data fusion
//    Serial.println();
//    transmit[2] = (uint8_t)(MPU.m_rawQuaternion[QUAT_W]>> 24);
//    transmit[3] = (uint8_t)(MPU.m_rawQuaternion[QUAT_W] >> 16);
//    transmit[4] = (uint8_t)(MPU.m_rawQuaternion[QUAT_W] >> 8);
//    transmit[5] = (uint8_t)(MPU.m_rawQuaternion[QUAT_W]);
//    transmit[6] = (uint8_t)(MPU.m_rawQuaternion[QUAT_X]>> 24);
//    transmit[7] = (uint8_t)(MPU.m_rawQuaternion[QUAT_X] >> 16);
//    transmit[8] = (uint8_t)(MPU.m_rawQuaternion[QUAT_X] >> 8);
//    transmit[9] = (uint8_t)(MPU.m_rawQuaternion[QUAT_X]);
//    transmit[10] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Y]>> 24);
//    transmit[11] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Y] >> 16);
//    transmit[12] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Y] >> 8);
//    transmit[13] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Y]);
//    transmit[14] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Z]>> 24);
//    transmit[15] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Z] >> 16);
//    transmit[16] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Z] >> 8);
//    transmit[17] = (uint8_t)(MPU.m_rawQuaternion[QUAT_Z]);
    f2b.f = MPU.m_fusedEulerPose[VEC3_Y];
    transmit[2] = f2b.b[3];
    transmit[3] = f2b.b[2];
    transmit[4] = f2b.b[1];
    transmit[5] = f2b.b[0];
    f2b.f = MPU.m_fusedEulerPose[VEC3_X];
    transmit[6] = f2b.b[3];
    transmit[7] = f2b.b[2];
    transmit[8] = f2b.b[1];
    transmit[9] = f2b.b[0];
    f2b.f = MPU.m_fusedEulerPose[VEC3_Z];
    transmit[10] = f2b.b[3];
    transmit[11] = f2b.b[2];
    transmit[12] = f2b.b[1];
    transmit[13] = f2b.b[0];
    Serial.write(transmit, 36);
  }
}
