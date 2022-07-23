#include "util/atomic.h"
#include "_constants.h"

void setup() {
  // serial
  Serial.begin(9600);
  Serial.println("INIT");

  pinMode(LED_BUILTIN, OUTPUT);

  // radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  
  // motors
  attachInterrupt(digitalPinToInterrupt(left_drive_motor.ENCA), read_drive_motor_encoder<LEFT_DRIVE_MOTOR>, RISING);
  attachInterrupt(digitalPinToInterrupt(right_drive_motor.ENCA), read_drive_motor_encoder<RIGHT_DRIVE_MOTOR>, RISING);
  attachInterrupt(digitalPinToInterrupt(back_drive_motor.ENCA), read_drive_motor_encoder<BACK_DRIVE_MOTOR>, RISING);
}

float test_duration = 0;
void loop() {
  current_time_micros = micros();
  float dt = ((float) (current_time_micros - prev_time_micros))/(1.0e6);
  prev_time_micros = current_time_micros;
//  left_drive_motor.set_drive_speed(0.2, dt);
//  right_drive_motor.set_drive_speed(0.2, dt);
//  back_drive_motor.set_drive_speed(0.2, dt);

  test_duration += dt;

  if (test_duration < 4.0) {
    drive_robot(0, 1, 0, dt);
  } else if (test_duration < 8.0) {
    drive_robot(0, -1, 0, dt);
  } else if (test_duration < 12.0) {
    drive_robot(1, 0, 0, dt);
  } else if (test_duration < 16.0) {
    drive_robot(-1, 0, 0, dt);
  } else {
    drive_robot(0, 0, 0, dt);
  }
}
