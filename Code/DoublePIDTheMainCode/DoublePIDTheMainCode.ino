//Viral Science www.youtube.com/c/viralscience  www.viralsciencecreativity.com
//Self Balancing Robot
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
double originalSetpoint = 173.95; // 181
int lineBetweenTwoPids = 7; // 15
//PID1
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
// 20 200 1.0
// 20 200 0.5
// 13 190 0.65
double Kp = 30;
double Ki = 315;
double Kd = 0.7;

int samplingTime = 7;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// PID2
// 80 550 1.0
// 95 550 0.85
double setpoint2 = originalSetpoint;
double movingAngleOffset2 = 0.1;
double input2, output2;
double Kp2 = 120;
double Ki2 = 550;
double Kd2 = 0.95;

int samplingTime2 = 7;
PID pid2(&input2, &output2, &setpoint2, Kp2, Ki2, Kd2, DIRECT);


double motorSpeedFactorLeft = 1.0;
double motorSpeedFactorRight = 1.0;

//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
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
   Serial.print(F("DMP Initialization failed (code "));
   Serial.print(devStatus);
   Serial.println(F(")"));
 }
}


void loop() {

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
        Ki -= 35;
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
        setpoint += 0.1;
        break;
      case '0':
        setpoint -= 0.1;
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
  Serial.println("Kp " + String(Kp) + " Ki " + String(Ki) + " Kd " + String(Kd) + " setpoint " + String(setpoint) + " d " + String(lineBetweenTwoPids) + " Kp2 " + String(Kp2) + " Ki2 " + String(Ki2) + " Kd2 " + String(Kd2));
        
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
    
    if(abs(input - originalSetpoint)>lineBetweenTwoPids)
      motorController.move(output2, MIN_ABS_SPEED);
    else
      motorController.move(output, MIN_ABS_SPEED);

    // a simple led  direction indicator
    if(input>originalSetpoint)
      digitalWrite(LED_BUILTIN, HIGH);
    else 
      digitalWrite(LED_BUILTIN, LOW); 

  }
}
