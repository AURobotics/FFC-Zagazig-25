#include <Arduino.h>
#include <Bluepad32.h>
#include <Servo.h>

ControllerPtr my_controllers[BP32_MAX_GAMEPADS];

#define STOP 0
#define MAX_SPEED 200
#define CONSTRAINT 30
#define FRONT_RIGHT_MOTOR 0
#define FRONT_LEFT_MOTOR 1
#define REAR_RIGHT_MOTOR 2
#define REAR_LEFT_MOTOR 3
#define PUMP 15
#define SERVO 23
#define SERVO_CENTER 90

Servo nozzle_servo;
bool pump_state = false;

struct motor_pins {
  int pin_in1;
  int pin_in2;
};

constexpr motor_pins motor_pins_list[4] = {
  { 19, 18 }, { 12, 14 }, { 17, 16 }, { 27, 26 }
};

void on_connected_controller(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    if (my_controllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      my_controllers[i] = ctl;
      return;
    }
  Serial.println("CALLBACK: Controller connected but no empty slot");
}

void on_disconnected_controller(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    if (my_controllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected, index=%d\n", i);
      my_controllers[i] = nullptr;
      return;
    }
  Serial.println("CALLBACK: Controller disconnected but not found");
}

void rotate_motor(int motor_number, int motor_speed) {
  int pwm = abs(motor_speed);
  if (motor_speed > 0) {
    analogWrite(motor_pins_list[motor_number].pin_in1, pwm);
    analogWrite(motor_pins_list[motor_number].pin_in2, 0);
  } else if (motor_speed < 0) {
    analogWrite(motor_pins_list[motor_number].pin_in1, 0);
    analogWrite(motor_pins_list[motor_number].pin_in2, pwm);
  } else {
    analogWrite(motor_pins_list[motor_number].pin_in1, 1024);
    analogWrite(motor_pins_list[motor_number].pin_in2, 1024);
  }
}

void process_car_movement(int input_value, int speed) {
  switch (input_value) {
    case 1:
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, speed);
      rotate_motor(REAR_RIGHT_MOTOR, speed);
      rotate_motor(REAR_LEFT_MOTOR, speed);
      break;
    case 2:
      rotate_motor(FRONT_RIGHT_MOTOR, -speed);
      rotate_motor(FRONT_LEFT_MOTOR, -speed);
      rotate_motor(REAR_RIGHT_MOTOR, -speed);
      rotate_motor(REAR_LEFT_MOTOR, -speed);
      break;
    case 3:
      rotate_motor(FRONT_RIGHT_MOTOR, -speed);
      rotate_motor(FRONT_LEFT_MOTOR, speed);
      rotate_motor(REAR_RIGHT_MOTOR, speed);
      rotate_motor(REAR_LEFT_MOTOR, -speed);
      break;
    case 4:
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, -speed);
      rotate_motor(REAR_RIGHT_MOTOR, -speed);
      rotate_motor(REAR_LEFT_MOTOR, speed);
      break;
    case 5:
      rotate_motor(FRONT_RIGHT_MOTOR, -speed);
      rotate_motor(FRONT_LEFT_MOTOR, speed);
      rotate_motor(REAR_RIGHT_MOTOR, -speed);
      rotate_motor(REAR_LEFT_MOTOR, speed);
      break;
    case 6:
      rotate_motor(FRONT_RIGHT_MOTOR, speed);
      rotate_motor(FRONT_LEFT_MOTOR, -speed);
      rotate_motor(REAR_RIGHT_MOTOR, speed);
      rotate_motor(REAR_LEFT_MOTOR, -speed);
      break;
    default:
      rotate_motor(FRONT_RIGHT_MOTOR, STOP);
      rotate_motor(FRONT_LEFT_MOTOR, STOP);
      rotate_motor(REAR_RIGHT_MOTOR, STOP);
      rotate_motor(REAR_LEFT_MOTOR, STOP);
      break;
  }
}

void setup_pin_modes() {
  for (const auto &motor : motor_pins_list) {
    pinMode(motor.pin_in1, OUTPUT);
    pinMode(motor.pin_in2, OUTPUT);
  }
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, LOW);
  nozzle_servo.attach(SERVO);
  nozzle_servo.write(SERVO_CENTER);
}

ControllerPtr return_first_controller() {
  for (auto controller : my_controllers)
    if (controller && controller->isConnected() && controller->hasData() && controller->isGamepad()) return controller;
  return nullptr;
}

void setup() {
  setup_pin_modes();
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  BP32.setup(&on_connected_controller, &on_disconnected_controller);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  if (!BP32.update()) return;
  ControllerPtr ctl = return_first_controller();
  if (ctl == nullptr) return;

  int right_stick = ctl->axisRX(), left_stick = ctl->axisX(), r2 = ctl->throttle(), l2 = ctl->brake();
  static int servo_angle = SERVO_CENTER;

  if (r2 > 0) process_car_movement(1, map(r2, 0, 1020, 0, MAX_SPEED));
  else if (l2 > 0) process_car_movement(2, map(l2, 0, 1020, 0, MAX_SPEED));
  else if (abs(left_stick) > 5) {
    int s = map(left_stick, -508, 512, -MAX_SPEED, MAX_SPEED);
    process_car_movement((s > 0) ? 3 : 4, abs(s));
  } else if (abs(right_stick) > 5) {
    int s = map(right_stick, -508, 512, -MAX_SPEED, MAX_SPEED);
    process_car_movement((s > 0) ? 5 : 6, abs(s));
  } else process_car_movement(0, 0);

  if (ctl->r1()) {
    servo_angle += 2;
    if (servo_angle > 180) servo_angle = 180;
    nozzle_servo.write(servo_angle);
    Serial.printf("Servo moving right: %d\n", servo_angle);
  } else if (ctl->l1()) {
    servo_angle -= 2;
    if (servo_angle < 0) servo_angle = 0;
    nozzle_servo.write(servo_angle);
    Serial.printf("Servo moving left: %d\n", servo_angle);
  }

  if (ctl->x()) {
    if (!pump_state) {
      digitalWrite(PUMP, HIGH);
      pump_state = true;
      Serial.println("Pump ON");
    }
  } else if (pump_state) {
    digitalWrite(PUMP, LOW);
    pump_state = false;
    Serial.println("Pump OFF");
  }
  delay(10);
}
