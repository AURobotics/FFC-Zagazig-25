#include <Arduino.h>

#define S1 2
#define S2 3
#define S3 4
#define S4 5
#define S5 6

#define ENA 7
#define IN1 8
#define IN2 9

#define ENB 10
#define IN3 11
#define IN4 12


float Kp = 40;
float Ki = 0.0;
float Kd = 20;

int baseSpeed = 100;

float error = 0;
float lastError = 0;
float integral = 0;

void setup() {
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
}

void loop() {
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  
  int weights[5] = {-2, -1, 0, 1, 2};
  int sensors[5] = {s1, s2, s3, s4, s5};

  int sum = 0, count = 0;
  
  for (int i = 0; i < 5; i++) {
    if (sensors[i] == LOW) {
      sum += weights[i];
      count++;
    }
  }

  if (count > 0) {
    error = (float)sum / count;
  } else {
    error = lastError;
  }

  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  setMotorSpeed(leftSpeed, rightSpeed);

  lastError = error;
}


void setMotorSpeed(int left, int right) {
  analogWrite(ENA, left);
  analogWrite(ENB, right);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
