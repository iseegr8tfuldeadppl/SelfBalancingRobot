
// Motors: imports, Initializtions
// https://create.arduino.cc/projecthub/electropeak/arduino-l293d-motor-driver-shield-tutorial-c1ac9b
#include <AFMotor.h> 
AF_DCMotor motor(1);
AF_DCMotor motor2(2);


// Gyroscope: Imports, Initializations
#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
int intgForceX=0, intgForceY=0, intgForceZ=0;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
int introtX=0, introtY=0, introtZ=0;


void setup() {

  // Serial: Setup
  Serial.begin(9600);

  // Gyroscope: Setup
  Wire.begin();
  setupMPU();
}

int incrementer = 0;
int totalXtemp = 0, totalYtemp = 0, totalZtemp = 0;
int totalX = 0, totalY = 0, totalZ = 0;
boolean forwardrequest = false;
boolean backwardrequest = false;
int initialized_time = 0;

int samples = 10;
int delayer = 5;
long requesttime;
long speeder = 110;

int tiniest_speed = 115;// default is 200
int forward = 20;
int backward = -28;
int tiniesttiming = 80;

int da9i9atsamt = 10;
int antispam = 1;

void loop() {
  algorithm_2_improved();
}


void algorithm_2_improved(){
      delay(delayer);
      if(forwardrequest){
          speeder = tiniest_speed + (totalX - forward);
          requesttime = abs(totalX*tiniesttiming/forward);
          
          motor2.setSpeed(speeder);
          motor.setSpeed(speeder);

          motor.run(BACKWARD);
          motor2.run(FORWARD);
          while(millis()<requesttime+initialized_time){
            delay(antispam);
          }
          forwardrequest = false;
          motor.run(RELEASE);
          motor2.run(RELEASE);
          delay(da9i9atsamt);
      } else if(backwardrequest){
          speeder = tiniest_speed + (-totalX + backward);
          requesttime = abs(totalX*tiniesttiming/backward);

          motor2.setSpeed(speeder);
          motor.setSpeed(speeder);
          
          motor.run(FORWARD);
          motor2.run(BACKWARD);
          while(millis()<requesttime+initialized_time){
            delay(antispam);
          }
          backwardrequest = false;
          motor.run(RELEASE);
          motor2.run(RELEASE);
          delay(da9i9atsamt);
      } else {
        if(samples==1){
              recordAccelRegisters();
              recordGyroRegisters();
          
              totalX = intgForceX;
              totalY = intgForceY;
              totalZ = intgForceZ;

              printData();
          
              // Motors: Write
              if(totalX>=forward){
                    forwardrequest = true;
                    initialized_time = millis();
              } else if(totalX<=backward){
                    backwardrequest = true;
                    initialized_time = millis();
              } else {
                    forwardrequest = false;
                    backwardrequest = false;
                    motor.run(RELEASE);
                    motor2.run(RELEASE);
              }
        } else {
            incrementer += 1;
            if(incrementer<=samples){
              // Gyroscope: Read
              recordAccelRegisters();
              recordGyroRegisters();
          
              totalXtemp += intgForceX;
              totalYtemp += intgForceY;
              totalZtemp += intgForceZ;
            }
            
            if(incrementer>=samples) {
              totalXtemp /= incrementer;
              totalYtemp /= incrementer;
              totalZtemp /= incrementer;
              incrementer = 0;
            }   
          
            if(incrementer==0){
              // Gyroscope: Display
              printData();
          
              // Motors: Write
              if(totalX>=forward){
                    forwardrequest = true;
                    initialized_time = millis();
              } else if(totalX<=backward){
                    backwardrequest = true;
                    initialized_time = millis();
              } else {
                    forwardrequest = false;
                    backwardrequest = false;
                    motor.run(RELEASE);
                    motor2.run(RELEASE);
              }
              totalX = totalXtemp;
              totalY = totalYtemp;
              totalZ = totalZtemp;
              totalXtemp = 0;
              totalYtemp = 0;
              totalZtemp = 0;
            }
        }
        
      }
}

void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}


void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read();
  processAccelData();
}


void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceX = gForceX*100;
  intgForceX = gForceX;

  gForceY = accelY / 16384.0;
  gForceY = gForceY*100;
  intgForceY = gForceY;
  
  gForceZ = accelZ / 16384.0;
  gForceZ = gForceZ*100;
  intgForceZ = gForceZ;
}


void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();
  gyroX = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();
  processGyroData();
}


void processGyroData(){
  rotX = gyroX / 131.0;
  introtX = rotX;
  rotY = gyroY / 131.0;
  introtY = rotY;
  rotZ = gyroZ / 131.0;
  introtZ = rotZ;
}


void printData(){
  Serial.print(totalX); Serial.print(" ");
  Serial.print(totalY); Serial.print(" ");
  Serial.print(totalZ); Serial.println(" ");
  //Serial.print(introtX); Serial.print(" ");
  //Serial.print(introtY); Serial.print(" ");
  //Serial.print(introtZ); Serial.println("");
}
