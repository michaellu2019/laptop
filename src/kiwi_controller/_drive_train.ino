void drive_robot(float vx, float vy, float w, float dt) {
  float motor_speeds[NUM_DRIVE_MOTORS];
  for (int i = 0; i < NUM_DRIVE_MOTORS; i++) {
    motor_speeds[i] = ik_drive_matrix[i][0] * vx + ik_drive_matrix[i][1] * vy + ik_drive_matrix[i][2] * w;
  }

  Serial.print(motor_speeds[0]);
  Serial.print(", ");
  Serial.print(motor_speeds[1]);
  Serial.print(", ");
  Serial.print(motor_speeds[2]);
  Serial.println();

  left_drive_motor.set_drive_speed(motor_speeds[0], dt);
  right_drive_motor.set_drive_speed(motor_speeds[1], dt);
  back_drive_motor.set_drive_speed(motor_speeds[2], dt);
}

template <int j>
void read_drive_motor_encoder() {
  int encb_val = digitalRead(drive_motors[j]->ENCB);
  if(encb_val > 0){
    drive_motors[j]->enc_posi++;
  } else{
    drive_motors[j]->enc_posi--;
  }
//  Serial.println(drive_motors[j]->enc_posi);
//
//  if (j == LEFT_DRIVE_MOTOR) {
//    left_drive_motor.enc_posi = drive_motors[j]->enc_posi;
//  } else if (j == RIGHT_DRIVE_MOTOR) {
//    right_drive_motor.enc_posi = drive_motors[j]->enc_posi;
//  } 
}
