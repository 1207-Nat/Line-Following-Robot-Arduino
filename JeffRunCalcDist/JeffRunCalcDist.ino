#include <LiquidCrystal.h>

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Motor control pins
const int ENA = 2;
const int IN1 = A1;
const int IN2 = A2;
const int ENB = 3;
const int IN3 = A3;
const int IN4 = A4;

const int RIGHT_SENSOR = A0;
const int LEFT_SENSOR = A1;

const int encoderPinA = 10;
const int encoderPinB = 11;

volatile int encoderPos = 0;
float distancePerRevolution = 0.2;

unsigned long startTime = 0;
unsigned long motorStartTime = 0;
unsigned long programDuration = 10000;
const unsigned long duration = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  lcd.begin(16, 2);
  for (int i = 0; i <= 2; i++) {
    lcd.clear();
    lcd.setCursor(i, 0);
    lcd.print("Press SELECT");
    delay(300);
  }
  lcd.setCursor(4, 1);
  lcd.print("to start");
}

char getKey() {
  if (analogRead(0) < 60) return 'A';
  if (analogRead(0) < 200) return 'B';
  if (analogRead(0) < 400) return 'C';
  if (analogRead(0) < 600) return 'D';
  if (analogRead(0) < 800) return 'E';
}

void moveForward() {
  analogWrite(ENA, 63);
  analogWrite(ENB, 68);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRightAngle() {
  const int turnSpeed = 255;
  analogWrite(ENA, 100);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  const int turnSpeed = 255;
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMoving() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void updateEncoder() {
  int b = digitalRead(encoderPinB);
  encoderPos += (digitalRead(encoderPinA) == b) ? 1 : -1;
}

void loop() {
  //if (!startTime) {
    char key = getKey();
    if (key == 'E') {
      startTime = millis();
      motorStartTime = millis();
      lcd.clear();
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - startTime;

      if (elapsedTime < duration) {
        analogWrite(ENA, 63 + (elapsedTime * 192 / duration));
        analogWrite(ENB, 68 + (elapsedTime * 187 / duration));
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      } else {
        int rightSensorValue = digitalRead(RIGHT_SENSOR);
        int leftSensorValue = digitalRead(LEFT_SENSOR);

        Serial.print("Right Sensor: ");
        Serial.print(rightSensorValue);
        Serial.print(" | Left Sensor: ");
        Serial.println(leftSensorValue);

        if (rightSensorValue == HIGH && leftSensorValue == HIGH) {
          moveForward();
        } else if (rightSensorValue == HIGH && leftSensorValue == LOW) {
          turnLeft();
        } else if (rightSensorValue == LOW && leftSensorValue == HIGH) {
          turnRightAngle();
        } else {
          stopMoving();
        }
      }

      if (elapsedTime >= programDuration) {
        stopMoving();
      }
        delay(1000);
      }
    }

  //if (startTime) {
    
  //}

  float revolutions = encoderPos / 360.0;
  float distance = revolutions * distancePerRevolution;

  lcd.setCursor(0, 1);
  lcd.print("            ");
  lcd.setCursor(10, 1);
  lcd.print(distance, 2);
