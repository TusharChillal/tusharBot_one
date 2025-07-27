#include <Arduino.h>
#include <ESP32Servo.h>

#define IN1 32 
#define IN2 33 
#define IN3 26 
#define IN4 27 
#define ENA 4 
#define ENB 23 


#define SERVO1_PIN 18
#define SERVO2_PIN 19

Servo servo1;
Servo servo2;

int left_PWM = 0;
int right_PWM = 0;
int targetAngle1 = 90;
int targetAngle2 = 90;

int currentAngle1 = 90;
int currentAngle2 = 90;

unsigned long lastServoUpdate = 0;
const int servoStepDelay = 10; 
const int servoStepSize = 1;   

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 ready to receive commands...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 1000, 2000);
  servo2.attach(SERVO2_PIN, 1000, 2000);

  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();

      if (input.startsWith("v")) {
        input.remove(0, 1);
        input.trim();

        int values[4];
        int i = 0;

        while (i < 4 && input.length() > 0) {
          int spaceIndex = input.indexOf(' ');
          String part = (spaceIndex == -1) ? input : input.substring(0, spaceIndex);
          values[i++] = part.toInt();
          input = (spaceIndex == -1) ? "" : input.substring(spaceIndex + 1);
        }

        if (i == 4) {
          left_PWM = values[0];
          right_PWM = values[1];
          targetAngle1 = constrain(values[2], 0, 180);
          targetAngle2 = constrain(values[3], 0, 180);

          Serial.printf("PWM L:%d R:%d | Servo1:%d Servo2:%d\n", left_PWM, right_PWM, targetAngle1, targetAngle2);

          motorControl(left_PWM, right_PWM);
        } else {
          Serial.println("Invalid command format. Expected 4 values.");
        }
      }

      input = "";
    } else {
      input += c;
    }
  }

  updateServoSmooth();
}

void updateServoSmooth() {
  unsigned long now = millis();
  if (now - lastServoUpdate >= servoStepDelay) {
    bool updated = false;

    if (currentAngle1 != targetAngle1) {
      currentAngle1 += (currentAngle1 < targetAngle1) ? servoStepSize : -servoStepSize;
      servo1.write(currentAngle1);
      updated = true;
    }

    if (currentAngle2 != targetAngle2) {
      currentAngle2 += (currentAngle2 < targetAngle2) ? servoStepSize : -servoStepSize;
      servo2.write(currentAngle2);
      updated = true;
    }

    if (updated) {
      lastServoUpdate = now;
    }
  }
}

void motorControl(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftPWM);
  }

  if (rightPWM >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightPWM);
  }
}
