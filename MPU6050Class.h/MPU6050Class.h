/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com

   Code refactored by: Tyler Chung
*/
#ifndef MPU6050Class_h
#define MPU6050Class_h

class MPU6050Class {
  struct XYZValues {
    double X;
    double Y;
    double Z;
  };
  public:
    struct XYZValues MPUOrientation;
    struct XYZValues accError;
    struct XYZValues gyroError;
    MPU6050Class(); //Constructor
    void begin(); //Initialize hardware
    void calcMPUOrientation(); //Calculate new MPU orientation angles
    void calibrateIMU(int iterations); //Calibrate to get error offsets of accelerometer and gyroscope

  private:
    const int MPU = 0x68;
    struct XYZValues accValues;
    struct XYZValues accAngles;
    struct XYZValues gyroValues;
    struct XYZValues gyroAngles;
    double elapsedTime, currentTime, previousTime;

    struct XYZValues getAccValues(); //Read the accelerometer data
    struct XYZValues getGyroValues(); //Read the gyroscope data
    struct XYZValues calcAccAngles(struct XYZValues newAccValues); //Calculate the accelerometer angles
    struct XYZValues applyFilter(struct XYZValues newAccAngles, struct XYZValues newGyroAngles); //Apply complimentary filter
};

#endif
