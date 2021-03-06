# define NUM_ARM_SERVOS 6

# define LEFT_ARM 0
# define RIGHT_ARM 1

class Arm {
  public:
    int servo_pins[NUM_ARM_SERVOS];
    int servo_vals[NUM_ARM_SERVOS];
    int servo_neutral_vals[NUM_ARM_SERVOS];
    int servo_low_vals[NUM_ARM_SERVOS];
    int servo_high_vals[NUM_ARM_SERVOS];
    VarSpeedServo servos[NUM_ARM_SERVOS];

    Arm(int servo_pins[], int servo_neutral_vals[], int servo_low_vals[], int servo_high_vals[]) {
      for (int i = 0; i < NUM_ARM_SERVOS; i++) {
        this->servo_pins[i] = servo_pins[i];
        this->servo_neutral_vals[i] = servo_neutral_vals[i]; 
        this->servo_low_vals[i] = servo_low_vals[i];
        this->servo_high_vals[i] = servo_high_vals[i];
      } 
    }

    initialize(int init_speed, int init_pause) {
      for (int i = 0; i < NUM_ARM_SERVOS; i++) {
        servos[i].attach(servo_pins[i]);
      }
      
      for (int i = 0; i < NUM_ARM_SERVOS; i++) {
        servos[i].write(servo_neutral_vals[i], init_speed);
        delay(init_pause);
      }
    }

    move_servo(int id, int servo_value, int servo_speed) {
      servos[id].write(servo_value, servo_speed);
    }
};

int arm_servo_pins[][NUM_ARM_SERVOS] = {{0, 0, 0, 0, 0, 0}, {2, 3, 4, 5, 6, 7}};
int arm_servo_neutral_vals[][NUM_ARM_SERVOS] = {{90, 90, 90, 90, 90, 90}, {90, 45, 90, 90, 90, 90}};
int arm_servo_low_vals[][NUM_ARM_SERVOS] = {{90, 90, 90, 90, 90, 90}, {0, 7, 0, 0, 0, 0}};
int arm_servo_high_vals[][NUM_ARM_SERVOS] = {{90, 90, 90, 90, 90, 90}, {180, 180, 180, 130, 180, 180}};
Arm right_arm = Arm(arm_servo_pins[RIGHT_ARM], arm_servo_neutral_vals[RIGHT_ARM], arm_servo_low_vals[RIGHT_ARM], arm_servo_high_vals[RIGHT_ARM]);
