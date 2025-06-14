/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com

   Code refactored by: Tyler Chung
*/
#include "MPU6050Class.h"
#include "Arduino.h"
#include <Wire.h>

//Constructor
MPU6050Class::MPU6050Class() {
  accError.X = 2.12;
  accError.Y = 3.46;
  gyroError.X = 2.05;
  gyroError.Y = 0.52;
  gyroError.Z = 2.50;
}

//Initialize hardware
void MPU6050Class::begin() {
  Wire.begin(); //Initialize comunication
  Wire.beginTransmission(MPU); //Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); //Talk to the register 6B
  Wire.write(0x00); //Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //End the transmission
}

//Called in the main looped used to calculate the MPU6050's orientation
void MPU6050Class::calcMPUOrientation() {
  //Get accelerometer readings, calculate the angles made by the accelerometer and apply calibration offsets to the angles
  accValues = getAccValues();
  accAngles = calcAccAngles(accValues);
  accAngles.X += accError.X;
  accAngles.Y += accError.Y;

  //Calculate the time passed each loop iteration
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime-previousTime) / 1000;

  //Get gyroscope readings and apply calibration offsets
  gyroValues = getGyroValues();
  gyroValues.X += gyroError.X;
  gyroValues.Y += gyroError.Y;
  gyroValues.Z += gyroError.Z;

  //Calculate gyroscope values converting angular velocity to angles
  gyroAngles.X += gyroValues.X*elapsedTime;
  gyroAngles.Y += gyroValues.Y*elapsedTime;
  gyroAngles.Z += gyroValues.Z*elapsedTime;

  //Apply complimentary filter to combine accelerometer and gyroscope data to reduce error
  MPUOrientation = applyFilter(accAngles, gyroAngles);
  gyroAngles.X = MPUOrientation.X; //Roll
  gyroAngles.Y = MPUOrientation.Y; //Pitch
  gyroAngles.Z = MPUOrientation.Z; //Yaw

  // Serial.print("Roll: ");
  // Serial.print(MPUOrientation.X);
  // Serial.print(" | Pitch: ");
  // Serial.print(MPUOrientation.Y);
  // Serial.print(" | Yaw: ");
  // Serial.println(MPUOrientation.Z);
}

//Grab accelerometer readings
struct MPU6050Class::XYZValues MPU6050Class::getAccValues() {
  struct XYZValues newAccValues;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();
   
  newAccValues.X = rawX / 16384.0;
  newAccValues.Y = rawY / 16384.0;
  newAccValues.Z = rawZ / 16384.0;
   
  return newAccValues;
}

//Grab gyroscope readings
struct MPU6050Class::XYZValues MPU6050Class::getGyroValues() {
  struct XYZValues newGyroValues;
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  newGyroValues.X = rawX / 131.0;
  newGyroValues.Y = rawY / 131.0;
  newGyroValues.Z = rawZ / 131.0;
   
  return newGyroValues;
}

//Calculate the accelerometer angles
struct MPU6050Class::XYZValues MPU6050Class::calcAccAngles(struct XYZValues newAccValues) {
  struct XYZValues newAccAngles;
  newAccAngles.X = ((atan((newAccValues.Y) / sqrt(pow((newAccValues.X), 2) + pow((newAccValues.Z), 2))) * 180 / PI));
  newAccAngles.Y = ((atan(-1 * (newAccValues.X) / sqrt(pow((newAccValues.Y), 2) + pow((newAccValues.Z), 2))) * 180 / PI));
  return newAccAngles;
}

//Complimentary filter to combine accelerometer and gyroscope data to reduce sensor drifting
struct MPU6050Class::XYZValues MPU6050Class::applyFilter(struct XYZValues newAccAngles, struct XYZValues newGyroAngles) {
  struct XYZValues newMPUOrientation;
  double complimentaryRatio = 0.96;
  newMPUOrientation.X = newGyroAngles.X*complimentaryRatio + newAccAngles.X*(1-complimentaryRatio); //Roll
  newMPUOrientation.Y = newGyroAngles.Y*complimentaryRatio + newAccAngles.Y*(1-complimentaryRatio); //Pitch
  newMPUOrientation.Z = newGyroAngles.Z; //Yaw
  return newMPUOrientation;
}

//Get the error offsets to calibrate the gyroscope and accelerometer readings so that on a flat surface 0 is being read
void MPU6050Class::calibrateIMU(int iterations) {
  struct XYZValues newAccValues, newAccAngles, newGyroValues;

  for (int i = 0; i <= iterations; i++) {
    //Calcualte values for accelerometer
    newAccValues = getAccValues();
    newAccAngles = calcAccAngles(newAccValues);
    accError.X += newAccAngles.X;
    accError.Y += newAccAngles.Y;

    //Calculate values for gyroscope
    newGyroValues = getGyroValues();
    gyroError.X += newGyroValues.X;
    gyroError.Y += newGyroValues.Y;
    gyroError.Z += newGyroValues.Z;
  }

  //Get average of accelerometer error readings
  accError.X = -accError.X / iterations;
  accError.Y = -accError.Y / iterations;

  //Get average of gyro error readings
  gyroError.X = -gyroError.X / iterations;
  gyroError.Y = -gyroError.Y / iterations;
  gyroError.Z = -gyroError.Z / iterations;

  // Serial.print("accErrorX: ");
  // Serial.print(accError.X);
  // Serial.print(" | accErrorY: ");
  // Serial.print(accError.Y);
  // Serial.print(" | gyroErrorX: ");
  // Serial.print(gyroError.X);
  // Serial.print(" | gyroErrorY: ");
  // Serial.print(gyroError.Y);
  // Serial.print(" | gyroErrorZ: ");
  // Serial.println(gyroError.Z);
}
