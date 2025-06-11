/*
  Auteur: Quentin Costermans & Zaid Ben
  Project: Line-following robot met obstakeldetectie & ontwijkingsmanoeuvres
  Voor: IoT 2025
  School: Thomas More - Graduaat IoT 2024-2025
*/

#include <Wire.h>
#include <AFMotor.h>
#include <Servo.h>

// IR sensoren voor lijnvolging
#define irLeft A0
#define irCenter A1
#define irRight A2

// Ultrasone sensoren voor afstandsmeting
#define trigPin 9
#define echoPin 10
#define trigPin2 A4
#define echoPin2 A5

// Knop voor pauzeren/starten
#define buttonPin A6

// Servo voor obstakel-scanning
#define servoPin A3

// Motoren
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
Servo scannerServo;

// Debounce logica & pauze status
bool paused = true;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 50;
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;

// Setup: initialisatie van pinnen, motoren en sensoren
void setup() {
  Serial.begin(9600);

  pinMode(irLeft, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  scannerServo.attach(servoPin);
  scannerServo.write(90); // beginpositie (recht vooruit)

  Serial.println("Setup complete. Waiting for button to start...");
}

// Main loop: bepaalt gedrag op basis van sensorinput
void loop() {
  checkButton();
  if (paused) {
    stopMotors();
    return;
  }

  if (obstacleDetected()) {
    Serial.println("🔴 Obstacle detected. Starting avoidance maneuver...");
    avoidSmartObstacle();
  } else {
    followLine();
  }
}

// Meet afstand in cm via ultrasone sensor
long readDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);
  long distance = duration * 0.034 / 2;
  Serial.print("Ultrasonic Distance (Trig: ");
  Serial.print(trig);
  Serial.print("): ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

// Draaien servo en meten afstand in een specifieke richting
long readScannerAtAngle(int angle) {
  scannerServo.write(angle);
  delay(600);
  long d = readDistance(trigPin2, echoPin2);
  Serial.print("📡 Scanner angle ");
  Serial.print(angle);
  Serial.print("° distance: ");
  Serial.print(d);
  Serial.println(" cm");
  return d;
}

// Detecteert of er een obstakel recht voor de robot staat
bool obstacleDetected() {
  long d = readDistance(trigPin, echoPin);
  return (d > 0 && d < 20);
}

// Obstacle ontwijking: slimme logica om obstakel te scannen en te omzeilen
void avoidSmartObstacle() {
  stopMotors(); delay(300);

  long leftDist = readScannerAtAngle(180);
  long rightDist = readScannerAtAngle(0);
  bool goLeft = leftDist > rightDist;
  readScannerAtAngle(90);

  Serial.print("✅ Chosen path: ");
  Serial.println(goLeft ? "LEFT" : "RIGHT");

  if (goLeft) {
    Serial.println("↪ Turning Left");
    turnLeft(); delay(500);
    scannerServo.write(0); delay(500);
  } else {
    Serial.println("↩ Turning Right");
    turnRight(); delay(500);
    scannerServo.write(180); delay(500);
  }

  stopMotors(); delay(300);

  while (true) {
    long sideScan = readDistance(trigPin2, echoPin2);
    if (sideScan > 40 && readDistance(trigPin2, echoPin2) > 20) {
      Serial.println("🟢 Detected end of obstacle.");
      moveForward();
      delay(400);
      break;
    }
    if (obstacleDetected()) {
      stopMotors(); delay(300);
      return;
    }
    Serial.println("🚙 Moving along the obstacle...");
    moveForward();
    delay(100);
  }

  stopMotors(); delay(300);
  Serial.println("🚗 Moving forward to bypass obstacle...");
  moveForward(); delay(600);
  stopMotors(); delay(300);

  if (goLeft) {
    Serial.println("↩ Turning to pass in front of obstacle (right turn)");
    turnRight(); delay(500);
  } else {
    Serial.println("↪ Turning to pass in front of obstacle (left turn)");
    turnLeft(); delay(500);
  }

  Serial.println("🚗 Moving forward to bypass obstacle...");
  moveForward(); delay(600);
  stopMotors(); delay(300);

  while (true) {
    long sideScan = readDistance(trigPin2, echoPin2);
    if (sideScan > 40 && readDistance(trigPin2, echoPin2) > 20) {
      Serial.println("🟢 Detected end of obstacle.");
      moveForward();
      delay(200);
      break;
    }

    if (obstacleDetected()) {
      stopMotors(); delay(300);
      exit;
    }

    Serial.println("🚙 Moving along the obstacle...");
    moveForward();
    delay(100);
  }

  Serial.println("🚗 Moving forward to bypass obstacle...");
  moveForward(); delay(600);
  stopMotors(); delay(300);

  if (goLeft) {
    Serial.println("↪ Final right turn to realign with path");
    turnRight(); delay(500);
  } else {
    Serial.println("↩ Final left turn to realign with path");
    turnLeft(); delay(500);
  }

  while (true) {
    if (digitalRead(irLeft) || digitalRead(irCenter) || digitalRead(irRight)) {
      break;
    }

    if (obstacleDetected()) {
      stopMotors(); delay(300);
      exit;
    }

    Serial.println("🚙 Moving along the obstacle...");
    moveForward();
    delay(100);
  }

  scannerServo.write(90); delay(500);
  Serial.println("✅ Returned to line-following position.");
  exit;
}

// Volgt een zwarte lijn m.b.v. IR sensoren
void followLine() {
  int left = digitalRead(irLeft);
  int center = digitalRead(irCenter);
  int right = digitalRead(irRight);

  Serial.print("IR Readings - L:");
  Serial.print(left);
  Serial.print(" C:");
  Serial.print(center);
  Serial.print(" R:");
  Serial.println(right);

  if (center && !left && !right) {
    Serial.println("→ Moving Forward (center on line)");
    moveForward();
  } else if (left && !center && !right) {
    Serial.println("↖ Adjust Left");
    turnLeft();
  } else if (right && !center && !left) {
    Serial.println("↗ Adjust Right");
    turnRight();
  } else if (center && left && !right) {
    Serial.println("↖ Slight Left");
    turnSlightLeft();
  } else if (center && right && !left) {
    Serial.println("↗ Slight Right");
    turnSlightRight();
  } else if (center && left && right) {
    Serial.println("⏸ Line: all sensors — pause/resume");
    pauseAndResume();
  } else {
    Serial.println("⚠ Lost Line - Stopping");
    stopMotors();
  }
}

// Bewegingen robot
void moveForward() {
  motor1.run(FORWARD); motor1.setSpeed(130);
  motor2.run(FORWARD); motor2.setSpeed(130);
}

void turnLeft() {
  motor1.run(FORWARD); motor1.setSpeed(150);
  motor2.run(BACKWARD); motor2.setSpeed(150);
}

void turnRight() {
  motor1.run(BACKWARD); motor1.setSpeed(150);
  motor2.run(FORWARD); motor2.setSpeed(150);
}

void turnSlightLeft() {
  motor1.run(FORWARD); motor1.setSpeed(120);
  motor2.run(BACKWARD); motor2.setSpeed(120);
}

void turnSlightRight() {
  motor1.run(BACKWARD); motor1.setSpeed(120);
  motor2.run(FORWARD); motor2.setSpeed(120);
}

void pauseAndResume() {
  Serial.println("⏸ Pausing 3s then nudging forward...");
  stopMotors(); delay(3000);
  moveSlightForward(); delay(1200);
}

void stopMotors() {
  Serial.println("🛑 Motors stopped.");
  motor1.run(RELEASE); motor1.setSpeed(0);
  motor2.run(RELEASE); motor2.setSpeed(0);
}

// Controleert of de start/pauze-knop werd ingedrukt
void checkButton() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        paused = !paused;
        Serial.print("▶ Robot is now ");
        Serial.println(paused ? "PAUSED" : "RUNNING");
        if (paused) stopMotors();
      }
    }
  }
  lastButtonState = reading;
}
