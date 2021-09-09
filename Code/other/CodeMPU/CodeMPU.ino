#include <Wire.h>

const int MPU_addr=0x68;
int16_t accX,accY,accZ,temp,gyroX, gyroY, gyroZ; 
long accXcal, accYcal, accZcal, gyroXcal, gyroYcal, gyroZcal, loopTimer, accTotal;
int cal; 
float anglePitch, anglePitchAcc, anglePitchOutput;
boolean setGyroAngles;
void setup() {
  Wire.begin();
  Serial.begin(9600);
  //pinMode(13, OUTPUT);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  //Configure the gyro (500dps full scale) 
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
  
  //digitalWrite(13,HIGH);
  // CALIBRATION!!
  /*for ( cal = 0; cal < 2000; cal++){ // old: 2000
      if ( cal % 200 == 0 ) Serial.print(".");
      read_MPU_data();
      accXcal += accX;
      accYcal += accY;
      accZcal += accZ;
      gyroXcal += gyroX;
      gyroYcal += gyroY;
      gyroZcal += gyroZ;
      delay(3);
    }*/
    
    Serial.println("Calibration completed !");
    // calculating the average offset
    accXcal /= 2000;
    accYcal /= 2000;
    accZcal /= 2000;
    gyroXcal /= 2000;
    gyroYcal /= 2000;
    gyroZcal /= 2000;

    Serial.print(""); Serial.print(accXcal);
    Serial.print(" "); Serial.print(accYcal);
    Serial.print(" "); Serial.print(accZcal);
    Serial.print(" "); Serial.print(gyroXcal);
    Serial.print(" "); Serial.print(gyroYcal);
    Serial.print(" "); Serial.println(gyroZcal);
    delay(2000);

    //digitalWrite(13,LOW);

    loopTimer = micros();
}

void loop() {
  read_MPU_data();

  //substracting the offset from the readings
  accX -= accXcal;
  accY -= accYcal;
  accZ -= accZcal;
  gyroX -= gyroXcal;
  gyroY -= gyroYcal;
  gyroZ -= gyroZcal;

  //gyro angle calculations

  float a = 1/(250.0*16.4);  // 0.0000611 = 1 / (250Hz * 65.5)
  anglePitch += gyroX * a ; // calculating the pitch angle and adding it to its variable

  // accelerometer angle calculations

  accTotal = sqrt((accX*accX)+(accY*accY)+(accZ*accZ)); //calculating the total accelero vector
  float c = 1/(3.142/180);
  anglePitchAcc = asin((float)accY/accTotal) * c ; // calculating the pitch angle and storing it in its variable

  // CALIBRATING VALUES:

  anglePitchAcc -= 0.0; // accelerometer calibration for pitch

  if (setGyroAngles) {    //IMU is already started
    anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004; //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  }else {                 //At first start
    anglePitch = anglePitchAcc;  //Set the gyro pitch angle equal to the accelerometer pitch angle 
    setGyroAngles = true;        //set the IMU started flag
  }

  // COMPLEMENTARY FILTER!
  // anglePitchOutput = 0.9 * anglePitchOutput + anglePitch * 0.1;  // //Take 90% of the output pitch value and add 10% of the raw pitch value
   float dt =  1 / 250; //sample period
   anglePitchOutput = 0.98 * ( anglePitchOutput + anglePitch*dt) + 0.02*anglePitchAcc;
   Serial.print("" + String(gyroX, DEC) + " " + String(accX, DEC));
   Serial.print(" " + String(gyroY, DEC) + " " + String(accY, DEC));
   Serial.println(" " + String(gyroZ, DEC) + " " + String(accZ, DEC));

  while(micros() - loopTimer < 4000);
  loopTimer = micros();
}
void read_MPU_data (){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  accX = Wire.read()<<8|Wire.read();  //0x3B & 0x3C                               //Add the low and high byte to the gyro_x variable
  accY = Wire.read()<<8|Wire.read();  //0x3D & 0x3E                               //Add the low and high byte to the gyro_y variable
  accZ = Wire.read()<<8|Wire.read();  //0x3F & 0x40
  temp = Wire.read()<<8|Wire.read();   //0x41 & 0x42 
  gyroX = Wire.read()<<8|Wire.read(); //0x43 & 0x44                                //Add the low and high byte to the gyro_x variable
  gyroY = Wire.read()<<8|Wire.read(); //0x45 & 0x46                                 //Add the low and high byte to the gyro_y variable
  gyroZ = Wire.read()<<8|Wire.read(); //0x47 & 0x48
  delay(100);
  }
