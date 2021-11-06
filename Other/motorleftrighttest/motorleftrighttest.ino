
#include <LMotorController.h>

double _motorAConst, _motorBConst;
LMotorController motorController(5, 6, 7, 8, 9, 10, 1.0, 1.0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motorController.reorient(0.5, 1.0);
  Serial.println(_motorAConst);
  Serial.println(_motorBConst);
  _motorAConst = 0.3;
  _motorBConst = 0.3;
  Serial.println(_motorAConst);
  Serial.println(_motorBConst);
  _motorAConst = motorController._motorAConst;
  _motorBConst = motorController._motorBConst;
  Serial.println(String(motorController._motorAConst) + " " + String(motorController._motorBConst));
  Serial.println(motorController._motorBConst);
  
}

void loop() {
  // put your main code here, to run repeatedly:
      
}
