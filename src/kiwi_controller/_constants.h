#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// radio
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
int data[5];

// drive motors
#define NUM_DRIVE_MOTORS 3

#define LEFT_DRIVE_MOTOR 0
#define RIGHT_DRIVE_MOTOR 1
#define BACK_DRIVE_MOTOR 2

long current_time_micros = 0;
long prev_time_micros = 0;

float ik_drive_matrix[NUM_DRIVE_MOTORS][NUM_DRIVE_MOTORS] = {{ -1.0/3.0, sqrt(3.0)/3.0, 1.0/3.0 }, 
                                                             { -1.0/3.0, -sqrt(3.0)/3.0, 1.0/3.0 }, 
                                                             { 2.0/3.0, 0, 1.0/3.0 }};

class PIDController {
  private:
    float kp;
    float ki;
    float kd;
    
    long error;
    float old_error;
    float d_error;
    float sum_error;
    
    long setpoint;
    long value;
    float dt;
    float output;

  public:
    PIDController(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
    }

    float get_controller_output(long setpoint, long value, float dt) {
      this->setpoint = setpoint;
      this->value = value;
      this->dt = dt;
      
      this->error = this->setpoint - this->value;
      this->sum_error = this->sum_error + this->error * this->dt;
      this->sum_error = 0;
      this->d_error = (this->error - this->old_error)/this->dt;
      this->old_error = this->error;

      this->output = this->kp * this->error + this->ki * this->sum_error + this->kd * this->d_error;

      // hack, pls fix
      if (abs(this->error) < 10) {
        return 0.0;  
      }

      return this->output;
    }
};

class DriveMotor {
  public:
    int ENCA;
    int ENCB;
    volatile long enc_posi;
    long enc_pos;
    long target_enc_pos;
    
  private:
    int id;
    int EN;
    int IN1;
    int IN2;

    float bal;
    PIDController* pid_controller;

    float pulses_per_rotation;
    float wheel_circumference;
    float target_angular_velocity; // min 0.125, max 3.3 rot/s

  public:
    DriveMotor(int id, int EN, int IN1, int IN2, int ENCA, int ENCB, 
                /*float kp, float ki, float kd,*/float bal, PIDController* pid_controller) {
      this->id = id;
      this->EN = EN;
      this->IN1 = IN1;
      this->IN2 = IN2;
      this->ENCA = ENCA;
      this->ENCB = ENCB;

      this->pid_controller = pid_controller;
      this->bal = bal;

      this->pulses_per_rotation = 64/4 * 50;
      this->wheel_circumference = 0.072 * PI;

      
      this->enc_posi = 0;
      this->enc_pos = 0;
      this->target_enc_pos = 0;
      
      pinMode(this->EN, OUTPUT);
      pinMode(this->IN1, OUTPUT);
      pinMode(this->IN2, OUTPUT);
      pinMode(this->ENCA, INPUT);
      pinMode(this->ENCB, INPUT);
    }
    
    void set_drive_speed(float angular_velocity, float dt) {
      this->target_angular_velocity = angular_velocity;
      float d_target = this->target_angular_velocity * dt * this->pulses_per_rotation;

      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        this->enc_pos = this->enc_posi;
      }
      this->target_enc_pos = (long) ceil(this->target_enc_pos + d_target);
//      this->target_enc_pos = -1600 * 2;
      
      int motor_pwm = (int) this->pid_controller->get_controller_output(this->target_enc_pos, this->enc_pos, dt);
      motor_pwm = constrain(motor_pwm, -255, 255);

//      Serial.print("T");
//      Serial.print(this->id);
//      Serial.print(" ");
//      Serial.print(this->target_enc_pos);
//      Serial.print(" E ");
//      Serial.print(this->enc_pos);
//      Serial.print(" M ");
//      Serial.print(motor_pwm);
//      Serial.println();

      write_pwm(motor_pwm);
    }

    void write_pwm(int motor_pwm) {
      int mapped_pwm = this->bal * motor_pwm;
    
      // control motor direction
      if (mapped_pwm < 0) {
        digitalWrite(this->IN1, HIGH);
        digitalWrite(this->IN2, LOW);
      } else if (mapped_pwm > 0) {
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, HIGH);
      } else {
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, LOW);
      }
    
      // set motor speed
      analogWrite(this->EN, abs(mapped_pwm));
    }
};

PIDController left_drive_motor_pid_controller = PIDController(2.5, 0.1, 0.28);
PIDController right_drive_motor_pid_controller = PIDController(2.5, 0.1, 0.28);
PIDController back_drive_motor_pid_controller = PIDController(2.5, 0.1, 0.28);

DriveMotor left_drive_motor = DriveMotor(LEFT_DRIVE_MOTOR, 10, 31, 30, 18, 19, 1.0, &left_drive_motor_pid_controller);
DriveMotor right_drive_motor = DriveMotor(RIGHT_DRIVE_MOTOR, 11, 33, 32, 20, 21, 1.0, &right_drive_motor_pid_controller);
DriveMotor back_drive_motor = DriveMotor(BACK_DRIVE_MOTOR, 5, 23, 22, 2, 3, 1.0, &back_drive_motor_pid_controller);
DriveMotor* drive_motors[NUM_DRIVE_MOTORS] = { &left_drive_motor, &right_drive_motor, &back_drive_motor };
