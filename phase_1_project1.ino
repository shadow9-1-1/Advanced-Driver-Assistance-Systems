#include <SoftwareSerial.h>
#include <Adafruit_LiquidCrystal.h>

// ---------------- LCD (I2C MODE) ----------------
Adafruit_LiquidCrystal lcd(0);  
// SDA → A4 , SCL → A5

// ---------------- BLUETOOTH HC-05 ----------------
SoftwareSerial BT(10, 11);  // TX, RX

// ---------------- MOTOR DRIVER PINS --------------
int ENA = 3;
int ENB = 9;
int IN1 = 5;
int IN2 = 4;
int IN3 = 7;
int IN4 = 6;

// ---------------- ULTRASONIC PINS ----------------
int trigPin = 12;
int echoPin = 8;

// ---------------- LDR (LIGHT SENSOR) -------------
int LDR = A0;    // controls headlights

// ---------------- HEADLIGHT PIN ------------------
int headlights = 2;   // use any pin you want

// ---------------- VARIABLES ----------------------
char btCmd = 'S';
int Speed = 0;
int currentSpeed = 0;
unsigned long lastUpdate = 0;

// TTC safety
const int headlightThreshold = 400;  // LDR darkness threshold
const float minSafeTTC = 1500;       // 1.5 seconds


// =================================================
//                SETUP
// =================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  lcd.begin(16, 2);
  lcd.print("  SMART  CAR");

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(headlights, OUTPUT);

  delay(1500);
  lcd.clear();
}



// =================================================
//                MAIN LOOP
// =================================================
void loop() {

  // ------------ Bluetooth Commands -----------------
  if (BT.available()) {
    btCmd = BT.read();
    Serial.print("CMD: ");
    Serial.println(btCmd);

    handleSpeedCommand(btCmd);
  }

  // ------------ Light Control by LDR ---------------
  int lightLevel = analogRead(LDR);
  if (lightLevel < headlightThreshold) {
    digitalWrite(headlights, HIGH);
  } else {
    digitalWrite(headlights, LOW);
  }

  // ------------ Ultrasonic Safety ------------------
  int dist = getDistance();
  float TTC = computeTTC(dist, currentSpeed);

  bool unsafe = (TTC > 0 && TTC < minSafeTTC);

  if (unsafe) {
    Speed = Speed * 0.4;   // slow down automatically
  }

  // ------------ Smooth speed update ----------------
  updateSpeed(Speed);

  // ------------ Direction Control ------------------
  moveCar(btCmd);

  // ------------ Update LCD -------------------------
  updateLCD(currentSpeed, lightLevel, dist, unsafe);
}



// =================================================
//               BLUETOOTH SPEED CONTROL
// =================================================
void handleSpeedCommand(char cmd) {
  switch (cmd) {
    case '0': Speed = 80; break;
    case '1': Speed = 100; break;
    case '2': Speed = 120; break;
    case '3': Speed = 140; break;
    case '4': Speed = 160; break;
    case '5': Speed = 180; break;
    case '6': Speed = 200; break;
    case '7': Speed = 220; break;
    case '8': Speed = 240; break;
    case '9': Speed = 255; break;
    case 'q': Speed = 255; break;
  }
}



// =================================================
//                ULTRASONIC FUNCTIONS
// =================================================
float  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);

  if (duration == 0) return 400;

  
  float d = duration * 0.034 / 2.0;
  if (d > 400) d = 400;
  return d;
}

float computeTTC(int distance, int speedPWM) {
  float speed_mps = (speedPWM / 255.0) * 1.0;  // assume 1 m/s max
  if (speed_mps <= 0) return -1;

  return (distance / 100.0) / speed_mps * 1000;  
}



// =================================================
//              SPEED DYNAMICS (SMOOTH)
// =================================================
void updateSpeed(int target) {

  if (millis() - lastUpdate < 40) return; // update every 40ms
  lastUpdate = millis();

  if (currentSpeed < target) currentSpeed += 5;
  else if (currentSpeed > target) currentSpeed -= 5;

  if (currentSpeed < 0) currentSpeed = 0;
  if (currentSpeed > 255) currentSpeed = 255;

  analogWrite(ENA, currentSpeed);
  analogWrite(ENB, currentSpeed);
}



// =================================================
//                  MOVEMENT
// =================================================
void moveCar(char cmd) {

  switch (cmd) {

    case 'F': // Forward
      forward();
      break;

    case 'B': // Backward
      backward();
      break;

    case 'L': // Left
      left();
      break;

    case 'R': // Right
      right();
      break;

    default: // Stop
      stopCar();
      break;
  }
}



void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}



// =================================================
//                   LCD DISPLAY
// =================================================
void updateLCD(int spd, int light, float dist, bool unsafe) {
  lcd.setCursor(0, 0);
  lcd.print("SPD:");
  lcd.print(spd);
  lcd.print(" LT:");
  lcd.print(light);

  lcd.setCursor(0, 1);

  if (unsafe) {
    lcd.print("TTC! D:");
    lcd.print(dist, 1);   // <-- FIXED
  } else {
    lcd.print("SAFE D:");
    lcd.print(dist, 1);   // <-- FIXED
  }
}