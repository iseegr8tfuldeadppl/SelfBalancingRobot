//Viral Science www.youtube.com/c/viralscience  www.viralsciencecreativity.com
//Self Balancing Robot
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

// bluetooth communication
SoftwareSerial mySerial(A2, A3); // RX, TX
String msg = "";

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 190.05; // 181
int lineBetweenTwoPids = 11.304; // 15
//PID1
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
// 20 200 1.0
// 20 200 0.5
// 13 190 0.65
double Kp = 58; // 30
double Ki = 780; // 280
double Kd = 0.95; // 0.8

int samplingTime = 7; // 7
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// PID2
// 80 550 1.0
// 95 550 0.85
double setpoint2 = originalSetpoint;
double movingAngleOffset2 = 0.1;
double input2, output2;
double Kp2 = 110; // 135
double Ki2 = 380; // 525
double Kd2 = 9.10; // 9.10

int samplingTime2 = 7;
PID pid2(&input2, &output2, &setpoint2, Kp2, Ki2, Kd2, DIRECT);


double motorSpeedFactorLeft = 1.0;
double motorSpeedFactorRight = 0.9;

//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
 mpuInterrupt = true;
}

float driving_delta=0.0;
boolean driving = false;
void treat_command(){
  //Serial.println("Received msg " + msg);

  if(msg=="")
    return;
  
  String command = getValue(msg, ' ', 0);
  if(command=="PID"){
    String index = getValue(msg, ' ', 1);
    double value = getValue(msg, ' ', 2).toDouble();
    if(index=="0"){
      Kp = value;
      pid.SetTunings(Kp, Ki, Kd);
    } else if(index=="1"){
      Ki = value;
      pid.SetTunings(Kp, Ki, Kd);
    } else if(index=="2"){
      Kd = value;
      pid.SetTunings(Kp, Ki, Kd);
      
    } else if(index=="3"){
      Kp2 = value;
      pid2.SetTunings(Kp2, Ki2, Kd2);
    } else if(index=="4"){
      Ki2 = value;
      pid2.SetTunings(Kp2, Ki2, Kd2);
    } else if(index=="5"){
      Kd2 = value;
      pid2.SetTunings(Kp2, Ki2, Kd2);
    }
  } else if(command=="B"){ // this is to set the BALANCE angle
    originalSetpoint = getValue(msg, ' ', 1).toDouble();
    setpoint = originalSetpoint;
  } else if(command=="S"){ // this is the angle delta at which we SWITCH from pid1 to pid2 (default: 7)
    lineBetweenTwoPids = getValue(msg, ' ', 1).toInt();
  } else if(command=="D"){ // this is the offset DELTA that is driven by the gyroscope of the phone
    driving_delta = getValue(msg, ' ', 2).toDouble();
  } else if(command=="DON"){ // this is to enable the offset DELTA that is driven by the gyroscope of the phone
    driving_delta = 0.0;
    driving = true;
  } else if(command=="DOFF"){ // this is to disable the offset DELTA that is driven by the gyroscope of the phone
    driving_delta = 0.0;
    driving = false;
  }
  
}


String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial.begin(115200);
  mySerial.begin(9600);
    
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0) {
   // turn on the DMP, now that it's ready
   mpu.setDMPEnabled(true);
  
   // enable Arduino interrupt detection
   attachInterrupt(0, dmpDataReady, RISING);
   mpuIntStatus = mpu.getIntStatus();
  
   // set our DMP Ready flag so the main loop() function knows it's okay to use it
   dmpReady = true;
  
   // get expected DMP packet size for later comparison
   packetSize = mpu.dmpGetFIFOPacketSize();
   
   //setup PID
   // PID1
   pid.SetMode(AUTOMATIC);
   pid.SetSampleTime(samplingTime); // 5 best 
   pid.SetOutputLimits(-255, 255); 
   
   // PID2
   pid2.SetMode(AUTOMATIC);
   pid2.SetSampleTime(samplingTime2); // 5 best 
   pid2.SetOutputLimits(-255, 255); 
 } else {
   // ERROR!
   // 1 = initial memory load failed
   // 2 = DMP configuration updates failed
   // (if it's going to break, usually the code will be 1)
   //Serial.print(F("DMP Initialization failed (code "));
   //Serial.print(devStatus);
   //Serial.println(F(")"));
 }
}


void loop() {
  if(mySerial.available()){
    char ah = mySerial.read();
    if(ah=='\n'){
      treat_command();
      msg = "";
    } else {
      msg.concat(ah);
    }
  }

  // apply driving delta if we're driving
  if(driving){
    setpoint = originalSetpoint + driving_delta;
  } else {
    driving_delta = 0.0;
    setpoint = originalSetpoint;
  }

  /*
  if(Serial.available()>0){
    char msg = Serial.read();
    switch(msg){
      case '4':
        Kp += 5;
        pid.SetTunings(Kp, Ki, Kd);
        break;
      case '1':
        Kp -= 5;
        if(Kp<0)
          Kp = 0;
        pid.SetTunings(Kp, Ki, Kd);
        break;
        
      case '5':
        Ki += 35;
        pid.SetTunings(Kp, Ki, Kd);
        break;
      case '2':
        Ki -= 35;
        if(Ki<0)
          Ki = 0;
        pid.SetTunings(Kp, Ki, Kd);
        break;
        
      case '6':
        Kd += 0.1;
        pid.SetTunings(Kp, Ki, Kd);
        break;
      case '3':
        Kd -= 0.1;
        if(Kd<0)
          Kd = 0;
        pid.SetTunings(Kp, Ki, Kd);
        break;


      case 'i':
        Kp2 += 5;
        pid2.SetTunings(Kp2, Ki2, Kd2);
        break;
      case 'k':
        Kp2 -= 5;
        if(Kp2<0)
          Kp2 = 0;
        pid2.SetTunings(Kp2, Ki2, Kd2);
        break;
        
      case 'o':
        Ki2 += 35;
        pid2.SetTunings(Kp2, Ki2, Kd2);
        break;
      case 'l':
        Ki2 -= 35;
        if(Ki2<0)
          Ki2 = 0;
        pid2.SetTunings(Kp2, Ki2, Kd2);
        break;
        
      case 'p':
        Kd2 += 0.1;
        pid2.SetTunings(Kp2, Ki2, Kd2);
        break;
      case 'm':
        Kd2 -= 0.1;
        if(Kd2<0)
          Kd2 = 0;
        pid2.SetTunings(Kp2, Ki2, Kd2);
        break;


      case '8':
        originalSetpoint += 0.1;
        setpoint = originalSetpoint;
        break;
      case '0':
        originalSetpoint -= 0.1;
        setpoint = originalSetpoint;
        break;
        
      case '9':
        lineBetweenTwoPids += 1;
        break;
      case '7':
        lineBetweenTwoPids -= 1;
        if(lineBetweenTwoPids<0)
          lineBetweenTwoPids = 0;
        break;
        
      case '\n':
        break;
    }
  }
  */
  
  // Step 0: do mpu stuff
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Step 1: calculate pid input
    input = ypr[1] * 180/M_PI + 180;
    input2 = input;

    // Step 2: calculate pid values
    pid.Compute();
    pid2.Compute();
    
    if(abs(input - setpoint)>lineBetweenTwoPids)
      motorController.move(output2, MIN_ABS_SPEED);
    else
      motorController.move(output, MIN_ABS_SPEED);

    // a simple led  direction indicator
    /*
    if(input>originalSetpoint)
      digitalWrite(LED_BUILTIN, HIGH);
    else 
      digitalWrite(LED_BUILTIN, LOW); 
    */

    //Serial.println(String(input) + " " + String(setpoint) + " | " + String(pid.GetKp()) + " " + String(pid.GetKi()) + " " + String(pid.GetKd()) + " | " + String(lineBetweenTwoPids) + " " + String(pid2.GetKp()) + " " + String(pid2.GetKi()) + " " + String(pid2.GetKd()));
    //mySerial.println(String(input) + " " + String(setpoint) + " | " + String(pid.GetKp()) + " " + String(pid.GetKi()) + " " + String(pid.GetKd()) + " | " + String(lineBetweenTwoPids) + " " + String(pid2.GetKp()) + " " + String(pid2.GetKi()) + " " + String(pid2.GetKd()));
    //mySerial.println(input);
  }
}
