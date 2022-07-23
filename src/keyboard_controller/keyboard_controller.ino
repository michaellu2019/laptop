#include "VarSpeedServo.h"
#include "_constants.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Init...");
  right_arm.initialize(30, 100);
}

void loop() {

}
