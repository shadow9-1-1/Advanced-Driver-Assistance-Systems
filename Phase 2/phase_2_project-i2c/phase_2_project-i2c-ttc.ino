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
int seatBeltSensor = A1;     
int doorLockSensor = A2;

// ---------------- VARIABLES ----------------------
char btCmd = 'S';
int Speed = 0;
int currentSpeed = 0;
unsigned long lastUpdate = 0;

// Bluetooth logging timer
unsigned long lastBTsend = 0;

// LDR threshold
const int headlightThreshold = 400;

// ---------------- TTC COLLISION AVOIDANCE --------
// Vehicle dynamics parameters
const float wheelDiameter = 0.065;      // meters (65mm typical for small robot car)
const float maxRPM = 200.0;             // max motor RPM at PWM 255
const float decelRate = 0.5;            // deceleration rate in m/s^2 (gradual braking)
const float minStoppingDistance = 5.0;  // minimum safe distance in cm

// Previous distance for velocity calculation
float prevDistance = 400.0;
unsigned long prevDistTime = 0;
float approachVelocity = 0.0;  // relative closing velocity in m/s


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

  // ------------ Calculate Approach Velocity --------
  calculateApproachVelocity(dist);

  // ------------ TTC-based Speed Adjustment ---------
  int adjustedSpeed = Speed;
  bool ttcWarning = false;
  
  if (btCmd == 'F') {
    adjustedSpeed = calculateTTCSpeed(dist, Speed);
    ttcWarning = (adjustedSpeed < Speed);
  }

  // ------------ UPDATE SPEED -----------------------
  updateSpeed(adjustedSpeed);

  // ------------ SAFETY CHECK -----------------------
  bool allowed = safetyOK();

  // ------------ Movement ---------------------------
  if (allowed) {
    
    // TTC system handles gradual deceleration - only full stop at minimum distance
    if (btCmd == 'F' && dist < minStoppingDistance) {
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
//        TTC (TIME-TO-COLLISION) SYSTEM
// =================================================

// Convert PWM speed to actual velocity in m/s
float pwmToVelocity(int pwmValue) {
  float rpm = (pwmValue / 255.0) * maxRPM;
  float wheelCircumference = PI * wheelDiameter;  // meters
  float velocity = (rpm * wheelCircumference) / 60.0;  // m/s
  return velocity;
}

// Calculate approach velocity (how fast obstacle is getting closer)
void calculateApproachVelocity(float currentDist) {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - prevDistTime;
  
  if (deltaTime >= 50) {  // Update every 50ms for stability
    float distChange = (prevDistance - currentDist) / 100.0;
    float timeSec = deltaTime / 1000.0;
    
    if (timeSec > 0) {
      // Positive value means obstacle is getting closer
      float newVelocity = distChange / timeSec;
      // Apply low-pass filter for smooth readings
      approachVelocity = 0.7 * approachVelocity + 0.3 * newVelocity;
    }
    
    prevDistance = currentDist;
    prevDistTime = currentTime;
  }
}

// Calculate TTC and determine safe speed
int calculateTTCSpeed(float distance, int targetSpeed) {
  // Convert distance to meters
  float distMeters = distance / 100.0;
  
  // Get current car velocity
  float carVelocity = pwmToVelocity(currentSpeed);
  
  // Total closing velocity = car velocity + approach velocity of obstacle
  float closingVelocity = carVelocity + approachVelocity;
  
  // division by zero case
  if (closingVelocity <= 0.01) {
    return targetSpeed;  
  }
  
  // Calculate Time-To-Collision (TTC)
  float ttc = distMeters / closingVelocity;
  
  // Calculate time needed to stop from current velocity
  float timeToStop = carVelocity / decelRate;
  
  // Calculate stopping distance
  float stoppingDistance = (carVelocity * carVelocity) / (2.0 * decelRate);
  
  float safetyMargin = 1.5;
  float requiredTTC = timeToStop * safetyMargin;
  float requiredDistance = stoppingDistance * safetyMargin * 100.0;  // back to cm
  
  // If TTC is less than required time to stop, reduce speed proportionally
  if (ttc < requiredTTC || distance < requiredDistance) {
    float ttcRatio = ttc / requiredTTC;
    float distRatio = distance / requiredDistance;
    
    float ratio = min(ttcRatio, distRatio);
    ratio = constrain(ratio, 0.0, 1.0);
    
    int newSpeed = (int)(targetSpeed * ratio);
    
    // Ensure minimum controllable speed or full stop at critical distance
    if (distance < minStoppingDistance) {
      return 0;
    }
    return max(newSpeed, 0);
  }
  
  return targetSpeed;
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
  if (spd < 100) lcd.print(" ");
  if (spd < 10) lcd.print(" ");
  lcd.print(spd);

  lcd.print(" L:");
  lcd.print(lightOn ? "ON " : "OFF");

  lcd.setCursor(0,1);
  if (!safe) {
    lcd.print("SAFETY FAIL    ");
  } 
  else if (dist < minStoppingDistance) {
    lcd.print("EMERG STOP!    ");
  }
  else if (currentSpeed < Speed && btCmd == 'F') {
    lcd.print("TTC BRAKING    ");
  }
  else {
    lcd.print("D:");
    lcd.print((int)dist);
    lcd.print("cm SAFE   ");
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

  BT.print("TARGET_SPD=");
  BT.println(Speed);

  BT.print("LIGHT=");
  BT.println(lightOn ? "ON" : "OFF");

  BT.print("DIST=");
  BT.println(dist);

  BT.print("APPROACH_VEL=");
  BT.println(approachVelocity, 3);

  BT.print("TTC_ACTIVE=");
  BT.println((currentSpeed < Speed && btCmd == 'F') ? "YES" : "NO");

  BT.print("SAFETY=");
  BT.println(safe ? "OK" : "FAIL");

  BT.print("CMD=");
  BT.println(btCmd);
}
