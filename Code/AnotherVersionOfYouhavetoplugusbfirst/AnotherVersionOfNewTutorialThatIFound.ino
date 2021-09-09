/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <PID_v1.h>
#include <LMotorController.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
#define MIN_ABS_SPEED 30



//PID
double originalSetpoint = -80; // -86.84
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 60;    //20
double Kd = 2.2;
double Ki = 270;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 9;  
int IN4 = 8;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);







// Timers
unsigned long timer = 0, start = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup()
{
  Serial.begin(115200);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);


  //setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  start = millis();
}


boolean done = true;
void loop()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeAccel();
  Vector raw = mpu.readRawAccel();

  // Calculate Pitch, Roll and Yaw

  pitch = pitch + norm.XAxis * timeStep;
  input = -((norm.XAxis+10)*5+30);
  //roll = roll + norm.XAxis * timeStep;
  //yaw = yaw + norm.ZAxis * timeStep;

  // Output raw

  /*
    while(Serial.available() > 0 && !done) {
      char msg = Serial.read();
      done = true;

      originalSetpoint = input;
      setpoint = originalSetpoint;
      PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
    }
  */

    Serial.println("yes");
    pid.Compute();
    //Serial.println("output " + String(output));
    motorController.move(-output, MIN_ABS_SPEED);
    Serial.println("norm.YAxis " + String(input) + " output " + String(output));
    delay((timeStep * 1000) - (millis() - timer));
}
