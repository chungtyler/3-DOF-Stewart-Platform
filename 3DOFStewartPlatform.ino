#include <Servo.h>
Servo servo1; //0 Degrees
Servo servo2; //120 Degrees
Servo servo3; //240 Degrees

struct servoAngles {
  double angle1;
  double angle2;
  double angle3;
};

struct servoAngles lastAngles = {45, 45, 45};

//Length Parameters in CM
double a = 2;
double b = 4;
double c = 3.75;
double d = 3.75;
double e = 4.25;////2 CM to 6 CM -> 179 Deg to 0 Deg
double theta = 0; //0 to 360 Degrees
double phi = 25; //-45 to 45 Degrees

void setup() {
  Serial.begin(9600);
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
}
double g = 0;
void loop() {
  // if (Serial.available() > 0) {
  //   phi = Serial.read();
  // }
  //g += 15;
  //e = 1*sin(radians(g)) + 4.2;
  //Serial.println(e);
  theta += 15;
  if (theta > 360*6) {
    theta = 0;
    sendServoCommands(theta, phi);
  } else if (theta > 360*3) {
    sendServoCommands(-theta, phi);
  } else {
    sendServoCommands(theta, phi);
  }
  Serial.println(theta);
  delay(30);
}

//Send New Servo Commands
// void sendServoCommands(struct servoAngles newServoAngles) {
//   double totalTime = 100;
//   controlServoSpeed(newServoAngles, totalTime, 15);
// }

//Control Speed of Servo Motors
// void controlServoSpeed(struct servoAngles newServoAngles, double totalTime, double timeStep) {
//   double angle1 = newServoAngles.angle1;
//   double angle2 = newServoAngles.angle2;
//   double angle3 = newServoAngles.angle3;

//   double angleStep1 = getAngleSteps(angle1, lastAngles.angle1, totalTime, timeStep);
//   double angleStep2 = getAngleSteps(angle2, lastAngles.angle2, totalTime, timeStep);
//   double angleStep3 = getAngleSteps(angle3, lastAngles.angle3, totalTime, timeStep);

//   double setAngle1, setAngle2, setAngle3;
//   for (int i; i*timeStep <= totalTime; i++) {
//     setAngle1 = lastAngles.angle1+i*angleStep1;
//     setAngle2 = lastAngles.angle2+i*angleStep2;
//     setAngle3 = lastAngles.angle3+i*angleStep3;

//     servo1.write(setAngle1);
//     servo2.write(setAngle2);
//     servo3.write(setAngle3);
//     delay(timeStep); //Miliseconds
//   }

//   servo1.write(angle1);
//   servo2.write(angle2);
//   servo3.write(angle3);

//   lastAngles.angle1 = angle1;
//   lastAngles.angle2 = angle2;
//   lastAngles.angle3 = angle3;
// }

// //Calculate the Angle Steps Per Loop for Speed Control of Servos
// double getAngleSteps(double angle, double lastAngle, double totalTime, double timeStep) {
//   double deltaAngle = angle-lastAngle;
//   double angleDirection = getSign(deltaAngle);
//   double steps = timeStep/totalTime;
//   double angleStep = deltaAngle/steps;
//   return angleDirection*angleStep;
// }

// //Calculate the Sign of a Value
// double getSign(double value) {
//   if (value > 0) {
//     return 1;
//   } else if (value < 0) {
//     return -1;
//   }
//   return 0;
// }

//Calcualte Crank Angles at 0, 120, and 240 Degrees
void sendServoCommands(double theta, double phi) {
  double angle1;
  double angle2;
  double angle3;

  angle1 = calcCrankAngle(theta, phi, 0)-7;
  angle2 = calcCrankAngle(theta, phi, 120);
  angle3 = calcCrankAngle(theta, phi, 240)-5;

  if ((angle1 == 0) || (angle2 == 0) || (angle3 == 0)) {
    angle1 = lastAngles.angle1;
    angle2 = lastAngles.angle2;
    angle3 = lastAngles.angle3;
  }

  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);

  Serial.print("Servo 1: ");
  Serial.print(angle1);
  Serial.print(" | Servo 2: ");
  Serial.print(angle2);
  Serial.print(" | Servo 3: ");
  Serial.println(angle3);

  lastAngles.angle1 = angle1;
  lastAngles.angle2 = angle2;
  lastAngles.angle3 = angle3;
}

//Theta: XY Angle, Phi: Z Angle
double calcCrankAngle(double theta, double phi, double psi) {
  double thetaRad = radians(theta);
  double phiRad = radians(phi);
  double psiRad = radians(psi);
  double gamma = atan(tan(phiRad)*cos(thetaRad-psiRad));

  double k1 = (pow(a,2)-pow(b,2)+pow(c,2)+pow(d,2)+pow(e,2))/(2*a*c);
  double k2 = (d*cos(gamma)+e*sin(gamma))/a;
  double k3 = (d/c)-cos(gamma);
  double k4 = 2*(sin(gamma)-(e/c));
  double A = k1-k2+k3;
  double B = k4;
  double C = k1-k2-k3;

  double alphaPrime = degrees(2*atan((-B+sqrt(pow(B,2)-4*A*C))/(2*A)));
  double alpha = int((alphaPrime-90)+360) % 360;
  return alpha;
}
