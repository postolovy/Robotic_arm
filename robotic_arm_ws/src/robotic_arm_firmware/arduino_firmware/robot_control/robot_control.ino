#include <Arduino.h>
#include <Servo.h>

#define dirPin 3
#define stepPin 5
#define servoShoulderPin 9   // was servoBasePin
#define servoElbowPin    10  // was servoArmPin
#define servoGripperPin  11  // was servoClawPin

#define SERVO_SHOULDER_START 90
#define SERVO_ELBOW_START    90
#define SERVO_GRIPPER_START  90

Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

int8_t idx = -1;                 // 0=stepper, 1=shoulder, 2=elbow, 3=gripper
uint8_t value_idx = 0;
char value[6] = "";             // optional sign + up to 4 digits + null terminator

void resetValueBuf() {
  value[0] = '\0';
  value_idx = 0;
}

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  servoShoulder.attach(servoShoulderPin);
  servoElbow.attach(servoElbowPin);
  servoGripper.attach(servoGripperPin);

  servoShoulder.write(SERVO_SHOULDER_START);
  servoElbow.write(SERVO_ELBOW_START);
  servoGripper.write(SERVO_GRIPPER_START);

  Serial.begin(115200);
  Serial.setTimeout(1);

  resetValueBuf();
}

void moveStepper(int steps) {
  // handle direction; HIGH/LOW can be swapped to match your driver
  if (steps >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
    steps = -steps;
  }

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(6000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(6000);
  }
}

void moveServo(int goal, Servo &motor) {
  int current = motor.read();
  
  if (goal > 180) goal = 180;
  if (goal < 0)   goal = 0;

  if (goal >= current) {
    for (int pos = current; pos <= goal; pos++) {
      motor.write(pos);
      // delay(10);
    }
  } else {
    for (int pos = current; pos >= goal; pos--) {
      motor.write(pos);
      // delay(10);
    }
  }
}

void loop() {
  if (!Serial.available()) return;

  char chr = Serial.read();

  // Select target: 'b' stepper base, 's' shoulder, 'e' elbow, 'g' gripper
  if (chr == 'b') { idx = 0; resetValueBuf(); }
  else if (chr == 's') { idx = 1; resetValueBuf(); }
  else if (chr == 'e') { idx = 2; resetValueBuf(); }
  else if (chr == 'g') { idx = 3; resetValueBuf(); }
  else if (chr == ',') {
    if (idx >= 0) {
      int val = atoi(value);
      if      (idx == 0) moveStepper(val);            // steps (can be signed)
      else if (idx == 1) moveServo(abs(val), servoShoulder);
      else if (idx == 2) moveServo(abs(val), servoElbow);
      else if (idx == 3) moveServo(abs(val), servoGripper);
    }
    resetValueBuf();
  }
  else if (chr == '-' && value_idx == 0) {
    value[value_idx++] = chr;
    value[value_idx] = '\0';
  }
  else if (isDigit(chr) && value_idx < 5) {
    value[value_idx++] = chr;
    value[value_idx] = '\0';
  }
  // ignore any other characters
}
