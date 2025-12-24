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
int LDR = A0;    

// ---------------- HEADLIGHT PIN ------------------
int headlights = 2;   

// ---------------- SAFETY SENSORS -----------------
int seatBeltSensor = A1;     // HIGH = Fastened
int doorLockSensor = A2;     // HIGH = Door Closed

// ---------------- VARIABLES ----------------------
char btCmd = 'S';
int Speed = 0;
int currentSpeed = 0;
unsigned long lastUpdate = 0;

// Bluetooth logging timer
unsigned long lastBTsend = 0;

// LDR threshold
const int headlightThreshold = 400;

// SAFE distance for forward
const int safeForwardDistance = 25;  // cm


// =================================================
//                SETUP
// =================================================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  lcd.begin(16, 2);
  lcd.print("   SMART  CAR");
  lcd.setCursor(0, 1);
  lcd.print("   By Shadow");
  delay(1500);
  lcd.clear();

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(headlights, OUTPUT);

  pinMode(seatBeltSensor, INPUT_PULLUP);
  pinMode(doorLockSensor, INPUT_PULLUP);
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
  bool lightOn = (lightLevel > headlightThreshold);
  digitalWrite(headlights, lightOn ? HIGH : LOW);

  // ------------ Ultrasonic Distance ----------------
  float dist = getDistance();

  // ------------ UPDATE SPEED -----------------------
  updateSpeed(Speed);

  // ------------ SAFETY CHECK -----------------------
  bool allowed = safetyOK();

  // ------------ Movement ---------------------------
  if (allowed) {
    
    // ONLY block FORWARD if obstacle close
    if (btCmd == 'F' && dist < safeForwardDistance) {
      stopCar();
    } else {
      moveCar(btCmd);
    }

  } else {
    stopCar();
  }

  // ------------ LCD -------------------------------
  updateLCD(currentSpeed, lightOn, dist, allowed);

  // ------------ Bluetooth Logging -----------------
  sendBluetoothLog(lightOn, dist, allowed);
}



// =================================================
//           SAFETY (SEATBELT & DOOR)
// =================================================
bool safetyOK() {
  bool beltOK = digitalRead(seatBeltSensor);  
  bool doorOK = digitalRead(doorLockSensor);  

  if (!beltOK || !doorOK)
    return false;

  return true;
}



// =================================================
//           BLUETOOTH SPEED CONTROL
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
//                ULTRASONIC
// =================================================
float getDistance() {
  digitalWrite(trigPin, LOW);
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



// =================================================
//              SPEED CONTROL
// =================================================
void updateSpeed(int target) {

  if (millis() - lastUpdate < 40) return;
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

    case 'F': forward(); break;
    case 'B': backward(); break;
    case 'L': left(); break;
    case 'R': right(); break;
    default: stopCar(); break;
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
//                   LCD
// =================================================
void updateLCD(int spd, bool lightOn, float dist, bool safe) {

  lcd.setCursor(0,0);
  lcd.print("SPD:");
  lcd.print(spd);
  lcd.print(" ");

  lcd.print("L:");
  lcd.print(lightOn ? "ON " : "OFF");

  lcd.setCursor(0,1);
  if (!safe) {
    lcd.print("SAFETY FAIL    ");
  } 
  else if (dist < safeForwardDistance) {
    lcd.print("OBSTACLE ALERT ");
  }
  else {
    lcd.print("STATUS SAFE    ");
  }
}



// =================================================
//        BLUETOOTH DATA LOGGING
// =================================================
void sendBluetoothLog(bool lightOn, float dist, bool safe) {
  
  if (millis() - lastBTsend < 500) return;
  lastBTsend = millis();

  BT.print("SPD=");
  BT.println(currentSpeed);

  BT.print("LIGHT=");
  BT.println(lightOn ? "ON" : "OFF");

  BT.print("DIST=");
  BT.println(dist);

  BT.print("SAFETY=");
  BT.println(safe ? "OK" : "FAIL");

  BT.print("CMD=");
  BT.println(btCmd);
}
