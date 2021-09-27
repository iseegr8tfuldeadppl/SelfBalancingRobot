#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1); // RX, TX
String msg = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  mySerial.begin(9600);
  Serial.println("READY");
  mySerial.println("READY");
}

void treat_command(){
  Serial.println("Received msg " + msg);
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
}
