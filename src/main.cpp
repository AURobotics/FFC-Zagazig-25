#include <Arduino.h> 
#include <PS4Controller.h>

#define STOP 0
#define MAX_SPEED 200
#define CONSTRAINT 30

#define FRONT_RIGHT_MOTOR 0
#define FRONT_LEFT_MOTOR 1
#define REAR_RIGHT_MOTOR 2
#define REAR_LEFT_MOTOR 3

struct motor_pins {
  int pin_en;
  int pin_in1;
  int pin_in2;
};

std::vector<motor_pins> motor_pins_list = {
  { 5, 17, 16 },   // front right
  { 13, 14, 27 },  // front left
  { 15, 4, 2 },    // rear right
  { 33, 26, 25 },  // rear left
};

void process_car_movement(int input_value, int speed) {
  switch (input_value) {
    case 1: // forward
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, speed);
      rotate_motor(REAR_RIGHT_MOTOR, speed);
      rotate_motor(REAR_LEFT_MOTOR, speed);
      break;

    case 2: // backward
      rotate_motor(FRONT_RIGHT_MOTOR, -speed);
      rotate_motor(FRONT_LEFT_MOTOR, -speed);
      rotate_motor(REAR_RIGHT_MOTOR, -speed);
      rotate_motor(REAR_LEFT_MOTOR, -speed);
      break;

    case 3: // right
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, speed);
      rotate_motor(REAR_RIGHT_MOTOR, speed);
      rotate_motor(REAR_LEFT_MOTOR, -speed);
      break;

    case 4: // left
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, -speed);
      rotate_motor(REAR_RIGHT_MOTOR, -speed);
      rotate_motor(REAR_LEFT_MOTOR, speed);
      break;

    case 5: //turn_right
      rotate_motor(FRONT_RIGHT_MOTOR, -speed);
      rotate_motor(FRONT_LEFT_MOTOR, speed);
      rotate_motor(REAR_RIGHT_MOTOR, -speed);
      rotate_motor(REAR_LEFT_MOTOR, speed);
      break;

    case 6: //turn_left
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, -speed);
      rotate_motor(REAR_RIGHT_MOTOR, speed);
      rotate_motor(REAR_LEFT_MOTOR, -speed);
      break;

    case 0: //stop
    default:
      rotate_motor(FRONT_RIGHT_MOTOR, STOP);
      rotate_motor(FRONT_LEFT_MOTOR, STOP);
      rotate_motor(REAR_RIGHT_MOTOR, STOP);
      rotate_motor(REAR_LEFT_MOTOR, STOP);
      break;
  }
}

void rotate_motor(int motor_number, int motor_speed) {
  if (motor_speed < 0) {
    digitalWrite(motor_pins_list[motor_number].pin_in1, LOW);
    digitalWrite(motor_pins_list[motor_number].pin_in2, HIGH);
  } else if (motor_speed > 0) {
    digitalWrite(motor_pins_list[motor_number].pin_in1, HIGH);
    digitalWrite(motor_pins_list[motor_number].pin_in2, LOW);
  } else {
    digitalWrite(motor_pins_list[motor_number].pin_in1, LOW);
    digitalWrite(motor_pins_list[motor_number].pin_in2, LOW);
  }
  analogWrite(motor_pins_list[motor_number].pin_en, abs(motor_speed));
}

void setup_pin_modes() {
  for (int i = 0; i < motor_pins_list.size(); i++) {
    pinMode(motor_pins_list[i].pin_in1, OUTPUT);
    pinMode(motor_pins_list[i].pin_in2, OUTPUT);
    pinMode(motor_pins_list[i].pin_en, OUTPUT);
  }
}

void setup() {
  setup_pin_modes();
  Serial.begin(115200);
  // Replace the "1a:2b:3c:01:01:01" with the MAC address
  PS4.begin("e8:c8:29:fc:43:d2");  // updated with masry ps4 controller mac address
  // "d0:7e:35:99:3f:7b" masry mac address
  // "e8:c8:29:fc:43:d2"
  Serial.println("Ready.");
}

void loop() {
  if (PS4.isConnected()) {
    int x = PS4.LStickX();
    // int y = PS4.LStickY();
    int z = PS4.RStickX();

    if (abs(x) > CONSTRAINT) {
      int speed = map(x, -128, 127, -MAX_SPEED, MAX_SPEED);
      if (speed > 0) {
        process_car_movement(3, speed);
        Serial.printf("right %d\n", speed);
      } else {
        process_car_movement(4, -speed);
        Serial.printf("left %d\n", -speed);
      }
    }
    if (abs(z) > CONSTRAINT) {
      int speed = map(z, -128, 127, -MAX_SPEED, MAX_SPEED);
      if (speed > 0) {
        process_car_movement(5, speed);
        Serial.printf("rotate right %d\n", speed);
      } else {
        process_car_movement(6, -speed);
        Serial.printf("rotate left %d\n", -speed);
      }
    }
    if (PS4.R2()) {
      int speed = map(PS4.R2Value(), -128, 127, -MAX_SPEED, MAX_SPEED);
      process_car_movement(1, speed);
      Serial.printf("Forward %d\n", speed);
    }
    if (PS4.L2()) {
      int speed = map(PS4.L2Value(), -128, 127, -MAX_SPEED, MAX_SPEED);
      process_car_movement(2, speed);
      Serial.printf("Backward %d\n", speed);
    }
    Serial.println();
    delay(10);
  }
}
// if (PS4.R1()) {
//   process_car_movement(4);
//   Serial.println("R1 Button");
// }
// if (PS4.L1()) {
//   process_car_movement(3);
//   Serial.println("L1 Button");
// }