/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2025-05-27
 * Version: 1.9
 * Description:
 *
 * Dieses ESP32-Programm ermöglicht einem Roboterfahrzeug autonomes
 * Fahren mit Hindernisvermeidung. Der autonome Modus (Vorwärtsfahrt)
 * wird über einen Bluepad32-kompatiblen Game-Controller (X-Taste/Quadrat)
 * gestartet. Die A-Taste (Kreuz) stoppt den Roboter. Zur Hindernis-
 * erkennung dient ein HC-SR04 Ultraschallsensor. Bei Unterschreitung
 * einer definierten Distanz stoppt der Roboter, dreht sich um einen
 * zufälligen Winkel (45-90 Grad) in eine zufällige Richtung und
 * setzt dann die Fahrt fort. Ein MPU6050 Gyrosensor misst den
 * Drehwinkel für das Ausweichmanöver. Ein L298N Motortreiber
 * (angesteuert via ESP32 LEDC PWM) steuert die Motoren.
 * Statusinformationen (Controller, Distanz, Zustand, Gyro) werden
 * auf einem SSD1306 I2C OLED-Display (180° gedreht) angezeigt.
 *
 * Hardware-Setup:
 * - ESP32 Mikrocontroller
 * - Bluepad32-kompatibler Game-Controller (z.B. PS4, PS5)
 * - L298N Motortreiber Modul
 * - Linker Motor: IN1 (GPIO 13), IN2 (GPIO 12)
 * - Rechter Motor: IN3 (GPIO 27), IN4 (GPIO 33)
 * - HC-SR04 Ultraschallsensor
 * - TRIG Pin: GPIO 25
 * - ECHO Pin: GPIO 26
 * - MPU6050 Gyroskop/Beschleunigungssensor (I2C)
 * - SSD1306 OLED Display (128x64, I2C, Adresse 0x3C)
 * - I2C SDA: Standard ESP32 I2C Pin (z.B. GPIO 21)
 * - I2C SCL: Standard ESP32 I2C Pin (z.B. GPIO 22)
 *
 * Controller-Tastenbelegung (Beispiel):
 * - X-Taste (Square): Startet autonomes Fahren / setzt Fahrt fort
 * - A-Taste (Cross): Stoppt alle Motoren / Roboter geht in IDLE
 *
 * Hinweis: Die Logik für driveForward() und rotateRobot() ist im Code
 * an eine spezifische (möglicherweise invertierte) Motor-Grunddrehrichtung
 * angepasst.
 ****************************************************/
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Bluepad32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstdlib> // Für random()

// HC-SR04 Pin Definitionen
#define PIN_TRIG 25
#define PIN_ECHO 26

// OLED Display Einstellungen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledInitialized = false;

// L298N Motor Driver INx Pin Definitions
#define PIN_M_L_IN1 13
#define PIN_M_L_IN2 12
#define PIN_M_R_IN3 27
#define PIN_M_R_IN4 33

// ESP32 LEDC PWM Config for INx Pins
#define CH_M_L_IN1 0
#define CH_M_L_IN2 1
#define CH_M_R_IN3 2
#define CH_M_R_IN4 3
#define PWM_FREQ_MOT 5000
#define PWM_RES_MOT 8
const int PWM_MAX_MOT = (1 << PWM_RES_MOT) - 1;

// MPU-6050 Sensor Objekt
Adafruit_MPU6050 mpu;

// Bluepad32 Controller Objekt
ControllerPtr myController = nullptr;

// Roboter Zustände
enum RobotState {
  IDLE,
  DRIVING_FORWARD,
  AVOIDING_OBSTACLE_ROTATING
};
RobotState currentState = IDLE;
String currentStateName = "IDLE"; // Für OLED-Anzeige

// Rotations- und Fahrlogik Variablen
float currentTurnProgressAngle = 0.0f; // Aktueller Fortschritt der Drehung
float targetRelativeTurnAngle = 0.0f; // Zielwinkel für die aktuelle Drehung
bool isTurningCW = false; // Drehrichtung für aktuelle Drehung

unsigned long lastTimeIntegration = 0;
unsigned long lastOledUpdateTime = 0;
unsigned long lastDistanceReadTime = 0;

const int FORWARD_PWM_SPEED = 100; 
const int ROTATION_PWM_SPEED = 80; 
const float TURN_DEADZONE_DEGREES = 5.0f; 
const float OBSTACLE_DISTANCE_CM = 30.0f; 
long currentDistanceCm = 0;

// --- Bluepad32 Connection Callbacks ---
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("Controller verbunden!");
    myController = ctl;
  } else {
    Serial.println("Weiterer Controller versucht zu verbinden, wird ignoriert.");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("Controller getrennt.");
    myController = nullptr;
    stopRobot();
    currentState = IDLE;
    currentStateName = "IDLE";
  }
}

// --- Motorsteuerungsfunktionen ---
void setMotorPower(int motorNum, int power) {
  uint8_t pinA_pwm_channel, pinB_pwm_channel;
  if (motorNum == 0) { pinA_pwm_channel = CH_M_L_IN1; pinB_pwm_channel = CH_M_L_IN2; }
  else { pinA_pwm_channel = CH_M_R_IN3; pinB_pwm_channel = CH_M_R_IN4; }
  power = constrain(power, -PWM_MAX_MOT, PWM_MAX_MOT);
  if (power > 0) { ledcWrite(pinA_pwm_channel, power); ledcWrite(pinB_pwm_channel, 0); }
  else if (power < 0) { ledcWrite(pinA_pwm_channel, 0); ledcWrite(pinB_pwm_channel, abs(power)); }
  else { ledcWrite(pinA_pwm_channel, 0); ledcWrite(pinB_pwm_channel, 0); }
}

void stopRobot() {
  Serial.println("DEBUG: stopRobot() aufgerufen.");
  setMotorPower(0, 0); setMotorPower(1, 0);
}

// ANGEPASST: Wenn Standardrichtung "rückwärts" ist, negieren wir die Power-Werte hier
void driveForward(int pwmSpeed) {
  Serial.print("DEBUG: driveForward(), Speed: "); Serial.println(pwmSpeed);
  setMotorPower(0, -pwmSpeed); // Negiert, um Vorwärtsfahrt zu erzielen
  setMotorPower(1, -pwmSpeed); // Negiert, um Vorwärtsfahrt zu erzielen
}

// ANGEPASST: Wenn die Grundrichtung der Motoren invertiert ist
void rotateRobot(bool counterClockwise, int pwmSpeed) {
  Serial.print("DEBUG: rotateRobot() CCW: "); Serial.print(counterClockwise);
  Serial.print(", Speed: "); Serial.println(pwmSpeed);
  
  // Annahme: setMotorPower(motor, positive_pwm) -> Fährt bei dir RÜCKWÄRTS
  // Annahme: setMotorPower(motor, negative_pwm) -> Fährt bei dir VORWÄRTS

  if (counterClockwise) { // Soll CCW drehen (z.B. Links rückwärts, Rechts vorwärts)
    // Linker Motor RÜCKWÄRTS (entspricht positive_pwm bei dir)
    // Rechter Motor VORWÄRTS (entspricht negative_pwm bei dir)
    setMotorPower(0, pwmSpeed);     // Links RÜCKWÄRTS
    setMotorPower(1, -pwmSpeed);    // Rechts VORWÄRTS
  } else { // Soll CW drehen (z.B. Links vorwärts, Rechts rückwärts)
    // Linker Motor VORWÄRTS (entspricht negative_pwm bei dir)
    // Rechter Motor RÜCKWÄRTS (entspricht positive_pwm bei dir)
    setMotorPower(0, -pwmSpeed);    // Links VORWÄRTS
    setMotorPower(1, pwmSpeed);     // Rechts RÜCKWÄRTS
  }
}

// --- HC-SR04 Distanzmessung ---
long readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 25000); // Timeout von 25ms (ca. 4m Reichweite)
  if (duration == 0) { // Timeout oder kein Echo
    return 999; // Einen hohen Wert zurückgeben, der als "kein Hindernis" oder "Fehler" interpretiert werden kann
  }
  long distance = duration / 58; // Ungefähre Umrechnung in cm
  return distance;
}

// --- OLED Display Funktion ---
void updateOLED() {
  if (!oledInitialized) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (myController && myController->isConnected()) {
    display.print("Ctrl: Verbunden");
  } else {
    display.print("Ctrl: Getrennt");
  }

  display.setCursor(0, 10);
  display.print("Dist: ");
  if (currentDistanceCm >= 999) { // Zeigt einen großen Wert bei Fehler/Timeout
      display.print("> "); display.print(OBSTACLE_DISTANCE_CM*2); 
  } else {
      display.print(currentDistanceCm);
  }
  display.print(" cm");

  display.setCursor(0, 20);
  display.print("Stat: "); display.print(currentStateName);

  display.setCursor(0, 30);
  if (currentState == AVOIDING_OBSTACLE_ROTATING) {
    display.print("Dreh: "); display.print(currentTurnProgressAngle, 0);
    display.print("/"); display.print(targetRelativeTurnAngle, 0); display.print(" G");
  } else {
    display.print("Ziel: Fahren");
  }

  display.setCursor(0, 40); 
  sensors_event_t a_event_oled, g_event_oled, temp_event_oled;
  mpu.getEvent(&a_event_oled, &g_event_oled, &temp_event_oled);
  float gyroZ_degS_oled = g_event_oled.gyro.z * (180.0f / M_PI);
  display.print("GyroZ: "); display.print(gyroZ_degS_oled, 1); display.print(" d/s");
  
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Roboter - Autonomes Fahren mit Hindernisvermeidung");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW); 

  randomSeed(micros()); 

  Wire.begin();

  Serial.println("Initialisiere OLED Display...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("Fehler: SSD1306 Allokierung fehlgeschlagen!"));
    oledInitialized = false;
  } else {
    oledInitialized = true;
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.setRotation(2);
    Serial.println("Display-Rotation auf 180 Grad gesetzt.");
    display.println("OLED OK"); display.display(); delay(1000);
  }

  Serial.println("Initialisiere MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Fehler: MPU6050 nicht gefunden!");
    if (oledInitialized) {
      display.clearDisplay(); display.setCursor(0, 0);
      display.println("MPU6050 FEHLER!"); display.println("Verkabelung pruefen.");
      display.display();
    }
    while (1) { delay(100); }
  }
  Serial.println("MPU6050 erfolgreich initialisiert.");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  delay(100);

  Serial.println("Initialisiere Motor PWM Kanäle...");
  ledcSetup(CH_M_L_IN1, PWM_FREQ_MOT, PWM_RES_MOT); ledcSetup(CH_M_L_IN2, PWM_FREQ_MOT, PWM_RES_MOT);
  ledcSetup(CH_M_R_IN3, PWM_FREQ_MOT, PWM_RES_MOT); ledcSetup(CH_M_R_IN4, PWM_FREQ_MOT, PWM_RES_MOT);
  Serial.println("Verbinde Motor Pins mit PWM Kanälen...");
  ledcAttachPin(PIN_M_L_IN1, CH_M_L_IN1); ledcAttachPin(PIN_M_L_IN2, CH_M_L_IN2);
  ledcAttachPin(PIN_M_R_IN3, CH_M_R_IN3); ledcAttachPin(PIN_M_R_IN4, CH_M_R_IN4);
  stopRobot();
  Serial.println("Motor PWM Initialisierung abgeschlossen.");

  Serial.println("Initialisiere Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  currentState = IDLE;
  currentStateName = "IDLE";
  Serial.println("Setup abgeschlossen. Warte auf Controller-Verbindung...");
  lastTimeIntegration = micros();
  if (oledInitialized) updateOLED();
}

void loop() {
  bool controllerDataUpdated = BP32.update();
  unsigned long currentTime = micros();

  // Controller-Eingaben verarbeiten
  if (myController && myController->isConnected() && controllerDataUpdated) {
    if (myController->x()) { // Quadrat-Taste
      if (currentState == IDLE) {
        Serial.println("Quadrat-Taste: Starte Vorwärtsfahrt.");
        currentState = DRIVING_FORWARD;
        currentStateName = "Fahren";
        driveForward(FORWARD_PWM_SPEED);
      }
    }
    if (myController->a()) { // X-Taste
      Serial.println("X-Taste: Stoppe Roboter.");
      stopRobot();
      currentState = IDLE;
      currentStateName = "IDLE";
    }
  } else if (myController == nullptr && (currentState != IDLE)) {
      Serial.println("Controller während Betrieb getrennt. Stoppe Roboter.");
      stopRobot();
      currentState = IDLE;
      currentStateName = "IDLE";
  }

  // Zustandslogik des Roboters
  if (currentState == DRIVING_FORWARD) {
    if (millis() - lastDistanceReadTime > 100) { // Distanz nicht in jeder Schleife messen
        currentDistanceCm = readDistanceCm();
        Serial.print("Distanz: "); Serial.print(currentDistanceCm); Serial.println(" cm");
        lastDistanceReadTime = millis();
    }

    if (currentDistanceCm < OBSTACLE_DISTANCE_CM && currentDistanceCm > 0) { // distance > 0 um Fehler von pulseIn zu ignorieren
      Serial.println("Hindernis erkannt!");
      stopRobot();
      
      targetRelativeTurnAngle = random(45, 91); // Zufälliger Winkel zwischen 45 und 90 Grad
      isTurningCW = random(0, 2); // 0 für CW, 1 für CCW (counterClockwise = true für CCW)
      
      Serial.print("Starte Ausweichdrehung: "); Serial.print(targetRelativeTurnAngle);
      Serial.print(" Grad "); Serial.println(isTurningCW ? "CW" : "CCW");

      currentTurnProgressAngle = 0.0f;
      lastTimeIntegration = currentTime;
      currentState = AVOIDING_OBSTACLE_ROTATING;
      currentStateName = "Ausweichen";
      rotateRobot(!isTurningCW, ROTATION_PWM_SPEED); // rotateRobot(true) ist CCW
    } else if (currentState == DRIVING_FORWARD) { 
        // Wenn kein Hindernis, sicherstellen, dass der Roboter fährt.
        // Dies ist wichtig, falls er z.B. durch Controller-Trennung gestoppt wurde und dann der Controller
        // wieder verbunden wird, während der State noch DRIVING_FORWARD ist, aber keine Motoren laufen.
        // Oder wenn der Start-Befehl kam, aber die Motoren aus irgendeinem Grund nicht gestartet sind.
        // Einfache Lösung: Einfach driveForward() erneut aufrufen, wenn er fahren soll.
        // Um ständige Aufrufe zu vermeiden, könnte man einen weiteren Flag "isMoving" nutzen,
        // aber für den Anfang sollte das so reichen oder man verlässt sich auf den Eintritt in den State.
        // Für dieses Modell: driveForward() wird beim Eintritt in den State aufgerufen.
        // Hier keine explizite Aktion, da er weiterfahren sollte.
    }
  } 
  else if (currentState == AVOIDING_OBSTACLE_ROTATING) {
    sensors_event_t a_event, g_event, temp_event; 
    mpu.getEvent(&a_event, &g_event, &temp_event); 
    float deltaTime = (currentTime - lastTimeIntegration) / 1000000.0f;
    lastTimeIntegration = currentTime;
    float gyroZ_radS = g_event.gyro.z; 

    currentTurnProgressAngle += abs(gyroZ_radS * (180.0f / M_PI)) * deltaTime;
    
    Serial.print("Drehung Fortschritt: "); Serial.print(currentTurnProgressAngle, 1);
    Serial.print("/"); Serial.print(targetRelativeTurnAngle,1); Serial.println(" Grad");

    if (currentTurnProgressAngle >= (targetRelativeTurnAngle - TURN_DEADZONE_DEGREES)) {
      Serial.println("Ausweichdrehung beendet.");
      stopRobot();
      currentState = DRIVING_FORWARD;
      currentStateName = "Fahren";
      delay(100); 
      driveForward(FORWARD_PWM_SPEED);
    }
  }

  // OLED Display aktualisieren (nicht in jeder Schleife, um Performance zu sparen)
  if (oledInitialized && (millis() - lastOledUpdateTime > 200)) { // z.B. alle 200ms
    updateOLED();
    lastOledUpdateTime = millis();
  }
  
  delay(10); // Kurze Pause für Stabilität
}
