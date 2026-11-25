/*
  Line-Following Ball Transport Robot  -  Arduino Mega 2560
  -----------------------------------------------------------
  AUTO mode:   read target colour -> follow line to the ball -> scoop DOWN over the
               matching ball -> drag it along the line to the station -> scoop UP
  MANUAL mode: phone app over HC-05 Bluetooth (Serial1, 9600 baud)
               F/B/L/R = drive, S = stop, 1 = scoop down, 0 = scoop up,
               A = auto mode, M = manual mode, ? = status

  Assumptions (change to match your track):
  - The ball spot and the station are marked by a cross line (all 5 IR sensors see black).
  - The colour sensor faces down/forward and can see the colour card, the ball and the
    station pad when the robot stops on a marker.
  - At start the robot sits on (or in front of) the target colour card.

  Libraries: Servo (built in), Wire (built in),
             Adafruit TCS34725 (Library Manager: "Adafruit TCS34725")
  Board:     Tools > Board > Arduino Mega or Mega 2560
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>

// ===================== PINS =====================
// TB6612FNG motor driver
const int PWMA = 5,  AIN1 = 22, AIN2 = 23;   // left motor
const int PWMB = 6,  BIN1 = 24, BIN2 = 25;   // right motor
const int STBY = 26;

// 5-channel IR line sensor, left to right
const int IR_PINS[5] = {A0, A1, A2, A3, A4};

// HC-SR04 ultrasonic
const int TRIG_PIN = 30;
const int ECHO_PIN = 31;

// Grabber servos
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

// Mode button (to GND) and status LED
const int MODE_BTN = 2;
const int LED_PIN  = 13;

// HC-05 on Serial1: Mega TX1 (18) -> divider -> HC-05 RX, Mega RX1 (19) <- HC-05 TX
// TCS34725 on I2C: SDA 20, SCL 21

// ===================== TUNING =====================
const int  IR_THRESHOLD  = 500;    // analog value between white and black
const bool LINE_IS_HIGH  = true;   // true if black gives a HIGH reading; flip if not
const int  BASE_SPEED    = 140;    // 0-255, line following speed
const int  DRAG_SPEED    = 110;    // slower while dragging, so the ball stays in the scoop
const int  MANUAL_SPEED  = 160;    // 0-255, phone control speed
const float KP = 35.0;             // line following: how hard to steer
const float KD = 20.0;             // line following: damping

const int UP_ANGLE    = 0;         // scoop raised (tune on the real robot)
const int DOWN_ANGLE  = 120;       // scoop lowered around the ball (tune on the real robot)
const int STEP_DELAY  = 8;         // ms per degree, smooth servo movement

const int OBSTACLE_CM   = 10;      // stop if something is closer than this (auto mode)
const int PASS_MARKER_MS = 250;    // drive this long to get off a cross line

// ===================== OBJECTS & STATE =====================
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo servo1, servo2;
int servoPos = UP_ANGLE;

enum Mode { MANUAL, AUTO };
Mode mode = MANUAL;                // start safe; press button or send 'A' to start

enum AutoState { READ_TARGET, TO_BALL, TO_STATION, DONE };
AutoState state = READ_TARGET;

enum Colour { NONE, RED, GREEN, BLUE, YELLOW };
Colour target = NONE;

float lastError = 0;
bool colourOk = false;

// ===================== HELPERS: COLOUR =====================
const char* colourName(Colour c) {
  switch (c) {
    case RED: return "RED";
    case GREEN: return "GREEN";
    case BLUE: return "BLUE";
    case YELLOW: return "YELLOW";
    default: return "NONE";
  }
}

// Rough colour guess from the sensor. Print the r/g/b values for your
// balls and pads, then adjust these numbers.
Colour readColour() {
  if (!colourOk) return NONE;
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 100) return NONE;                 // too dark or nothing there

  float rf = (float)r / c, gf = (float)g / c, bf = (float)b / c;
  Serial.print("Colour r="); Serial.print(rf, 2);
  Serial.print(" g="); Serial.print(gf, 2);
  Serial.print(" b="); Serial.println(bf, 2);

  if (rf > 0.45 && gf < 0.35)                 return RED;
  if (rf > 0.38 && gf > 0.35 && bf < 0.25)    return YELLOW;
  if (gf > rf && gf > bf)                     return GREEN;
  if (bf > rf && bf > gf)                     return BLUE;
  return NONE;
}

// ===================== HELPERS: MOTORS =====================
void setMotor(int pwm, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); speed = -speed; }
  analogWrite(pwm, speed);
}

void drive(int left, int right) {
  setMotor(PWMA, AIN1, AIN2, left);
  setMotor(PWMB, BIN1, BIN2, right);
}

void stopMotors() { drive(0, 0); }

// ===================== HELPERS: SCOOP =====================
void moveScoop(int targetAngle) {
  int dir = (targetAngle > servoPos) ? 1 : -1;
  while (servoPos != targetAngle) {
    servoPos += dir;
    servo1.write(servoPos);
    servo2.write(180 - servoPos);   // mirrored: both servos turn the scoop the same way
    delay(STEP_DELAY);
  }
}

// ===================== HELPERS: SENSORS =====================
bool onBlack(int i) {
  int v = analogRead(IR_PINS[i]);
  return LINE_IS_HIGH ? (v > IR_THRESHOLD) : (v < IR_THRESHOLD);
}

long distanceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long t = pulseIn(ECHO_PIN, HIGH, 20000);   // 20 ms timeout
  if (t == 0) return 999;                    // no echo: treat as clear
  return t * 0.0343 / 2;
}

// One step of line following. Returns true when a cross line (marker) is found.
bool followLineStep(int speed) {
  const int weight[5] = {-2, -1, 0, 1, 2};
  int sum = 0, count = 0;
  for (int i = 0; i < 5; i++) {
    if (onBlack(i)) { sum += weight[i]; count++; }
  }

  if (count == 5) return true;               // cross line: ball spot or station

  float error;
  if (count == 0) error = (lastError > 0) ? 3 : -3;   // lost line: turn back toward it
  else            error = (float)sum / count;

  float correction = KP * error + KD * (error - lastError);
  lastError = error;

  drive(speed + correction, speed - correction);
  return false;
}

void passMarker(int speed) {
  drive(speed, speed);
  delay(PASS_MARKER_MS);
}

// ===================== BLUETOOTH & BUTTON =====================
void report(const char* msg) {
  Serial.println(msg);
  Serial1.println(msg);
}

void setMode(Mode m) {
  mode = m;
  stopMotors();
  if (m == AUTO) { state = READ_TARGET; report("Mode: AUTO"); }
  else           { report("Mode: MANUAL"); }
}

void handleBluetooth() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;

    if (c == 'A') { setMode(AUTO); continue; }
    if (c == 'M') { setMode(MANUAL); continue; }
    if (c == '?') {
      report(mode == AUTO ? "AUTO" : "MANUAL");
      report(colourName(target));
      continue;
    }

    if (mode != MANUAL) continue;            // drive commands only in manual mode

    switch (c) {
      case 'F': drive(MANUAL_SPEED, MANUAL_SPEED);   break;
      case 'B': drive(-MANUAL_SPEED, -MANUAL_SPEED); break;
      case 'L': drive(-MANUAL_SPEED, MANUAL_SPEED);  break;
      case 'R': drive(MANUAL_SPEED, -MANUAL_SPEED);  break;
      case 'S': stopMotors();                        break;
      case '1': stopMotors(); moveScoop(DOWN_ANGLE); report("Scoop down"); break;
      case '0': stopMotors(); moveScoop(UP_ANGLE);   report("Scoop up");   break;
    }
  }
}

void handleButton() {
  static bool lastPressed = false;
  bool pressed = (digitalRead(MODE_BTN) == LOW);
  if (pressed && !lastPressed) {
    delay(30);                               // debounce
    if (digitalRead(MODE_BTN) == LOW) setMode(mode == AUTO ? MANUAL : AUTO);
  }
  lastPressed = pressed;
}

// ===================== AUTONOMOUS RUN =====================
void runAuto() {
  static unsigned long lastCheck = 0;

  // Obstacle check every 100 ms
  if (millis() - lastCheck > 100) {
    lastCheck = millis();
    if (state != DONE && state != READ_TARGET && distanceCm() < OBSTACLE_CM) {
      stopMotors();
      return;
    }
  }

  switch (state) {
    case READ_TARGET:
      stopMotors();
      delay(300);
      target = readColour();
      if (target == NONE) {                  // try again until a colour is seen
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(200);
        return;
      }
      digitalWrite(LED_PIN, HIGH);
      report("Target colour:");
      report(colourName(target));
      state = TO_BALL;
      break;

    case TO_BALL:
      if (followLineStep(BASE_SPEED)) {      // at a ball marker
        stopMotors();
        delay(200);
        if (readColour() == target) {        // matching ball here
          moveScoop(DOWN_ANGLE);             // lower the scoop around the ball
          report("Scoop down, dragging ball");
          state = TO_STATION;
          passMarker(DRAG_SPEED);            // move on slowly with the ball
        } else {
          passMarker(BASE_SPEED);            // wrong colour, keep looking
        }
      }
      break;

    case TO_STATION:
      if (followLineStep(DRAG_SPEED)) {      // at a station marker
        stopMotors();
        delay(200);
        if (readColour() == target) {        // matching station
          moveScoop(UP_ANGLE);               // lift the scoop, ball stays at the station
          report("Scoop up, ball at station");
          drive(-BASE_SPEED, -BASE_SPEED);   // back away so the scoop clears the ball
          delay(300);
          stopMotors();
          state = DONE;
          return;
        }
        passMarker(DRAG_SPEED);
      }
      break;

    case DONE:
      stopMotors();
      digitalWrite(LED_PIN, (millis() / 500) % 2);   // slow blink = finished
      break;
  }
}

// ===================== SETUP & LOOP =====================
void setup() {
  Serial.begin(115200);     // USB, for debugging
  Serial1.begin(9600);      // HC-05

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Write the up angle before attaching, so the scoop doesn't jerk at power-on
  servo1.write(UP_ANGLE);
  servo2.write(180 - UP_ANGLE);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  colourOk = tcs.begin();
  report(colourOk ? "Colour sensor OK" : "Colour sensor NOT found");
  report("Ready. Send A for auto, M for manual.");
}

void loop() {
  handleButton();
  handleBluetooth();
  if (mode == AUTO) runAuto();
}
