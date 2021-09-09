
#include <PID_v1.h>
#include <LMotorController.h>



#define MIN_ABS_SPEED 30



//PID
double originalSetpoint = -80;
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


#define SWITCH 2

// Gyroscope
#include "Wire.h"
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup() {
  pinMode(SWITCH, INPUT);
  // put your setup code here, to run once:

  // Gyroscope
  Wire.begin();
  Serial.begin(9600);
  check_I2c(MPU_addr); // Check that there is an MPU
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  
  //setup PID
  Serial.println("PID setup...");
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
  Serial.println("Doneof setup");
}

void loop() {
  // put your main code here, to run repeatedly:

  //timer = millis();
  recordGyroRegisters();
  printData();
  input = -(AcY / 150 + 80);
  Serial.println("input " + String(input));
  pid.Compute();
  motorController.move(-output, MIN_ABS_SPEED); //-output
}


void printData(){
  //output = String(AcX) + " " + String(AcY) + " " + String(AcZ) + " " + String(GyX) + " " + String(GyY) + " " + String(GyZ);
  //Serial.println(output);
  //Serial.print(AcX); Serial.print(" ");
  //Serial.print(AcY); Serial.print(" ");
  //Serial.print(AcZ); Serial.print(" ");
  //Serial.print(GyX); Serial.print(" ");
  //Serial.print(GyY); Serial.print(" ");
  //Serial.print(GyZ); //Serial.print(" ");
  //Serial.print(digitalRead(5)); Serial.print(" ");
  //Serial.print(digitalRead(3));  Serial.print(" ");
  //Serial.print("0"); Serial.println("");
}



void recordGyroRegisters(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

byte check_I2c(byte addr){
  // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  byte error;
  Wire.beginTransmission(addr);
  error = Wire.endTransmission();
   
  if (error == 0){
    Serial.print(" Device Found at 0x");
    Serial.println(addr,HEX);
  } else{
    Serial.print(" No Device Found at 0x");
    Serial.println(addr,HEX);
  }
  return error;
}
