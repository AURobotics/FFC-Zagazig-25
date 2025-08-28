#include <Arduino.h>
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define STOP 0
#define MAX_SPEED 200
#define CONSTRAINT 30

#define FRONT_RIGHT_MOTOR 0
#define FRONT_LEFT_MOTOR 1
#define REAR_RIGHT_MOTOR 2
#define REAR_LEFT_MOTOR 3

struct motor_pins
{
  // int pin_en;
  int pin_in1;
  int pin_in2;
};

constexpr motor_pins motor_pins_list[4] = {
    {19, 18}, // front right
    {12, 14}, // front left
    {17, 16}, // rear right
    {27, 26}, // rear left
};

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl)
{
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
  {
    if (myControllers[i] == nullptr)
    {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot)
  {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl)
{
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
  {
    if (myControllers[i] == ctl)
    {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController)
  {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void rotate_motor(int motor_number, int motor_speed)
{
  int pwm = abs(motor_speed);
  if (motor_speed > 0) // forward
  {

    analogWrite(motor_pins_list[motor_number].pin_in1, pwm);
    analogWrite(motor_pins_list[motor_number].pin_in2, 0);
  }
  else if (motor_speed < 0) // backward
  {
    analogWrite(motor_pins_list[motor_number].pin_in1, 0);
    analogWrite(motor_pins_list[motor_number].pin_in2, pwm);
  }
  else // stop
  {
    analogWrite(motor_pins_list[motor_number].pin_in1, 1024);
    analogWrite(motor_pins_list[motor_number].pin_in2, 1024);
  }
}

void process_car_movement(int input_value, int speed)
{
  switch (input_value)
  {
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
    rotate_motor(FRONT_RIGHT_MOTOR, -speed);
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

  case 5: // rotate_right
    rotate_motor(FRONT_RIGHT_MOTOR, -speed);
    rotate_motor(FRONT_LEFT_MOTOR, speed);
    rotate_motor(REAR_RIGHT_MOTOR, -speed);
    rotate_motor(REAR_LEFT_MOTOR, speed);
    break;

  case 6: // rotate_left
    rotate_motor(FRONT_RIGHT_MOTOR, speed);
    rotate_motor(FRONT_LEFT_MOTOR, -speed);
    rotate_motor(REAR_RIGHT_MOTOR, speed);
    rotate_motor(REAR_LEFT_MOTOR, -speed);
    break;

  case 0: // stop
  default:
    rotate_motor(FRONT_RIGHT_MOTOR, STOP);
    rotate_motor(FRONT_LEFT_MOTOR, STOP);
    rotate_motor(REAR_RIGHT_MOTOR, STOP);
    rotate_motor(REAR_LEFT_MOTOR, STOP);
    break;
  }
}

void setup_pin_modes()
{
  for (const auto &motor_pins : motor_pins_list)
  {
    pinMode(motor_pins.pin_in1, OUTPUT);
    pinMode(motor_pins.pin_in2, OUTPUT);
    // pinMode(motor_pins.pin_en, OUTPUT);
  }
}

ControllerPtr returnFirstController()
{
  for (auto myController : myControllers)
  {
    if (myController && myController->isConnected() && myController->hasData())
    {
      if (myController->isGamepad())
      {
        return myController;
      }
    }
  }
  return nullptr;
}

void setup()
{
  setup_pin_modes();
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

void loop()
{
  bool controllerUpdated = BP32.update();
  if (!controllerUpdated)
    return;

  ControllerPtr ctl = returnFirstController();
  if (ctl == nullptr)
    return;

  // --- Joysticks ---
  int left_stick = ctl->axisX();   // Left stick X [-508 --> 512]
  int right_stick = ctl->axisRX(); // Right stick X [-508 --> 512]

  // --- Triggers ---
  int l2 = ctl->brake();    // L2 trigger [0 --> 1020]
  int r2 = ctl->throttle(); // R2 trigger [0 --> 1020]

  // --- Handle forward (R2 trigger) ---
  if (r2 > 0)
  {
    int speed_forward = map(r2, 0, 1020, 0, MAX_SPEED);
    process_car_movement(1, speed_forward);
    Serial.printf("Forward %d\n", speed_forward);
  }

  // --- Handle backward (L2 trigger) ---
  if (l2 > 0)
  {
    int speed_backward = map(l2, 0, 1023, 0, MAX_SPEED);
    process_car_movement(2, speed_backward);
    Serial.printf("Backward %d\n", speed_backward);
  }

  // --- Handle left stick (X axis, left/right) ---
  if (left_stick > 5 || left_stick < 0)
  {
    int speed_horizontal = map(left_stick, -508, 512, -MAX_SPEED, MAX_SPEED);
    if (speed_horizontal > 0)
    {
      process_car_movement(3, speed_horizontal); // right
      Serial.printf("Right %d\n", speed_horizontal);
    }
    else
    {
      process_car_movement(4, -speed_horizontal); // left
      Serial.printf("Left %d\n", -speed_horizontal);
    }
  }

  // // --- Handle right stick (rotation) ---
  if (right_stick > 5 || right_stick < 0)
  {
    int speed_rotate = map(right_stick, -508, 512, -MAX_SPEED, MAX_SPEED);
    if (speed_rotate > 0)
    {
      process_car_movement(5, speed_rotate); // right
      Serial.printf("Rotate Right %d\n", speed_rotate);
    }
    else
    {
      process_car_movement(6, -speed_rotate); // left
      Serial.printf("Rotate Left %d\n", -speed_rotate);
    }
  }
  delay(10);
}

// R2 0 --> 1020
// L2 0 --> 1020
// Lstick -508 --> 512 (5 deadzone)
// Rstick -508-->512 (5 deadzone)
