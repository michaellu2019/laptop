# define NUM_ARM_SERVOS 6
# define NUM_JOINTS_PER_ARM 7
# define NUM_ARMS 2

# define LEFT_ARM 0
# define RIGHT_ARM 1

class Endeffector {
  public:
    int solenoid_pin;
    int t;
    int open_duration;
    int close_duration;
    float trigger_val;
    enum TriggerState { OPEN, CLOSED, READY };
    TriggerState trigger_state;

    Endeffector(int solenoid_pin, float trigger_val) {
      this->solenoid_pin = solenoid_pin;
      this->t = 0;
      this->open_duration = 100;
      this->close_duration = 500;
      this->trigger_val = trigger_val;
      this->trigger_state = READY;
    }

    initialize() {
      pinMode(this->solenoid_pin, OUTPUT);
    }

    open() {
      if (this->trigger_state == READY) {
        digitalWrite(this->solenoid_pin, HIGH);
        this->t = 0;
        this->trigger_state = OPEN;
      }
    }

    tick() {
      this->t++;
      if (this->trigger_state == OPEN && this->t > this->open_duration) {
        digitalWrite(this->solenoid_pin, LOW);
        this->trigger_state = CLOSED;
      } else if (this->trigger_state == CLOSED && this->t > this->close_duration + this->open_duration) {
        this->trigger_state = READY;
      }
    }

    handle_pose_data(float data) {
      this->t++;
      if (this->trigger_state == READY && data > this->trigger_val) {
        digitalWrite(this->solenoid_pin, HIGH);
        this->t = 0;
        this->trigger_state = OPEN;
      } else if (this->trigger_state == OPEN && this->t > this->open_duration) {
        digitalWrite(this->solenoid_pin, LOW);
        this->trigger_state = CLOSED;  
      } else if (this->trigger_state == CLOSED && this->t > this->close_duration + this->open_duration) {
        this->trigger_state = READY;
      }
    }
};

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
        servos[i].write(servo_neutral_vals[i], init_speed);
      }
      
      for (int i = 0; i < NUM_ARM_SERVOS; i++) {
        servos[i].attach(servo_pins[i]);
        delay(init_pause);
      }
    }

    move_servo(int id, int servo_value, int servo_speed) {
      if (servo_value < this->servo_low_vals[id] || servo_value > this->servo_high_vals[id])
        return;
      
      servos[id].write(servo_value, servo_speed);
    }
};

int arm_servo_pins[NUM_ARMS][NUM_ARM_SERVOS] = {{14, 3, 4, 5, 6, 7}, {0, 0, 0, 0, 0, 0}};
int arm_servo_joint_state_offset_vals[NUM_ARMS][NUM_ARM_SERVOS] = {{90, 135, 90, 0, 90, 90}, {90, 90, 90, 90, 90, 90}};
float arm_servo_joint_state_coefficient_vals[NUM_ARMS][NUM_ARM_SERVOS] = {{1.0, -1.0, -2.0/3.0, 1.0, -2.0/3.0, -1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
int arm_servo_neutral_vals[NUM_ARMS][NUM_ARM_SERVOS] = {{90, 135, 90, 0, 90, 90}, {90, 90, 90, 90, 90, 90}};
int arm_servo_low_vals[NUM_ARMS][NUM_ARM_SERVOS] = {{0, 7, 0, 0, 0, 0}, {90, 90, 90, 90, 90, 90}};
int arm_servo_high_vals[NUM_ARMS][NUM_ARM_SERVOS] = {{180, 180, 180, 130, 180, 180}, {90, 90, 90, 90, 90, 90}};

Arm left_arm = Arm(arm_servo_pins[LEFT_ARM], arm_servo_neutral_vals[LEFT_ARM], arm_servo_low_vals[LEFT_ARM], arm_servo_high_vals[LEFT_ARM]);

int arm_endeffector_pins[NUM_ARMS] = {22, 0};
float arm_endeffector_joint_state_coefficient_vals[NUM_ARMS] = {-1000.0, -1000.0};
float arm_endeffector_joint_state_trigger_vals[NUM_ARMS] = {14.0, 14.0};

Endeffector left_endeffector(arm_endeffector_pins[LEFT_ARM], arm_endeffector_joint_state_trigger_vals[LEFT_ARM]);
