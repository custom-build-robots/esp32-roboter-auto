/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2025-05-27
 * Version: 1.9
 * Description:
 *
 * Dieses ESP32-Programm steuert ein Roboterfahrzeug, das sich
 * mittels eines Bluepad32-kompatiblen Game-Controllers zu einer
 * vorgegebenen Himmelsrichtung (N, O, S, W) ausrichten kann.
 * Die absolute Ausrichtung wird durch einen QMC5883L Kompass-Sensor
 * ermittelt. Ein MPU6050 liefert zusätzlich Gyroskopdaten (Drehrate).
 * Ein L298N Motortreiber (angesteuert über ESP32 LEDC PWM)
 * kontrolliert die Fahrmotoren. Bei Knopfdruck (X-Taste des
 * Controllers) dreht der Roboter, bis die aktuelle Kompassausrichtung
 * dem Ziel (innerhalb einer Toleranz) entspricht. Statusinfos wie
 * Controller-Verbindung, Kompass-Heading, Zielrichtung und Gyro-Daten
 * werden auf einem SSD1306 I2C OLED-Display (180° gedreht)
 * dargestellt. Die A-Taste des Controllers stoppt die aktuelle Bewegung.
 * Eine korrekte Kalibrierung des QMC5883L ist für präzise
 * Ergebnisse entscheidend.
 *
 * Hardware-Setup:
 * - ESP32 Mikrocontroller
 * - Bluepad32-kompatibler Game-Controller (z.B. PS4, PS5)
 * - L298N Motortreiber Modul
 * - Linker Motor: IN1 (GPIO 13), IN2 (GPIO 12)
 * - Rechter Motor: IN3 (GPIO 27), IN4 (GPIO 33)
 * - QMC5883L Magnetometer/Kompass-Sensor (I2C)
 * - MPU6050 Gyroskop/Beschleunigungssensor (I2C)
 * - SSD1306 OLED Display (128x64, I2C, Adresse 0x3C)
 * - I2C SDA: Standard ESP32 I2C Pin (z.B. GPIO 21)
 * - I2C SCL: Standard ESP32 I2C Pin (z.B. GPIO 22)
 *
 * Controller-Tastenbelegung (Beispiel):
 * - X-Taste (Square): Startet die Ausrichtung zur Ziel-Himmelsrichtung
 * - A-Taste (Cross): Stoppt alle Motoren / Bricht Ausrichtung ab
 * - Ziel-Himmelsrichtung: Standard 'N', kann im Code geändert werden
 * (targetCardinalDirection)
 *
 ****************************************************/
#include <Wire.h>
#include <QMC5883LCompass.h>  // NEU: Bibliothek für QMC5883L
#include <Adafruit_Sensor.h>   // Wird weiterhin für MPU6050 benötigt
#include <Adafruit_MPU6050.h>  // Für das Gyroskop MPU-6050
#include <Bluepad32.h>
#include <Adafruit_GFX.h>      // Für OLED Display
#include <Adafruit_SSD1306.h>  // Für OLED Display

// ----------------------------
// OLED Display Einstellungen
// ----------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledInitialized = false;

// ----------------------------
// Ziel-Himmelsrichtung
// ----------------------------
char targetCardinalDirection = 'N';
float targetHeadingDegrees = 0.0f;

// ----------------------------
// L298N Motor Driver INx Pin Definitions
// ----------------------------
#define PIN_M_L_IN1 13
#define PIN_M_L_IN2 12
#define PIN_M_R_IN3 27
#define PIN_M_R_IN4 33

// ----------------------------
// ESP32 LEDC PWM Config
// ----------------------------
#define CH_M_L_IN1 0
#define CH_M_L_IN2 1
#define CH_M_R_IN3 2
#define CH_M_R_IN4 3
#define PWM_FREQ_MOT 5000
#define PWM_RES_MOT 8
const int PWM_MAX_MOT = (1 << PWM_RES_MOT) - 1;

// ----------------------------
// Sensor Objekte
// ----------------------------
QMC5883LCompass qmc;          // NEU: Objekt für QMC5883LCompass
Adafruit_MPU6050 mpu;

// ----------------------------
// Bluepad32 Controller Objekt
// ----------------------------
ControllerPtr myController = nullptr;

// ----------------------------
// Programm-Zustände
// ----------------------------
enum RobotState {
  IDLE,
  ROTATING_TO_TARGET
};
RobotState currentState = IDLE;

// ----------------------------
// Rotationslogik Variablen
// ----------------------------
float currentHeadingQMC = 0.0f;
float currentGyroZ_degS = 0.0f;
unsigned long lastSensorReadTime = 0;
unsigned long lastOledUpdateTime = 0;

const int ROTATION_PWM_SPEED = 80;
const float DEADZONE_DEGREES = 5.0f;

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
    if (currentState == ROTATING_TO_TARGET) {
      Serial.println("Rotation gestoppt aufgrund Controller-Trennung.");
      stopRobot();
      currentState = IDLE;
    }
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

void rotateRobot(bool counterClockwise, int pwmSpeed) {
  Serial.print("DEBUG: rotateRobot() CCW: "); Serial.print(counterClockwise);
  Serial.print(", Speed: "); Serial.println(pwmSpeed);
  if (counterClockwise) { setMotorPower(0, -pwmSpeed); setMotorPower(1, pwmSpeed); }
  else { setMotorPower(0, pwmSpeed); setMotorPower(1, -pwmSpeed); }
}

// --- Hilfsfunktionen ---
float getTargetAngleFromCardinal(char cardinal) {
  switch (cardinal) {
    case 'N': return 0.0f; case 'O': return 90.0f;
    case 'S': return 180.0f; case 'W': return 270.0f;
    default: return 0.0f;
  }
}

float normalizeAngle180(float angle) {
  angle = fmod(angle, 360.0f);
  if (angle > 180.0f) angle -= 360.0f;
  else if (angle <= -180.0f) angle += 360.0f;
  return angle;
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
  display.print("Head: ");
  display.print(currentHeadingQMC, 1);
  display.print(" G");

  display.setCursor(0, 20);
  display.print("Stat: ");
  if (currentState == ROTATING_TO_TARGET) {
    display.print("Dreht...");
  } else {
    display.print("Bereit");
  }
  
  display.setCursor(0, 30);
  display.print("Ziel: ");
  display.print(targetCardinalDirection);
  display.print(" (");
  display.print(targetHeadingDegrees, 0);
  display.print(" G)");

  display.setCursor(0, 40);
  display.print("GyroZ: ");
  display.print(currentGyroZ_degS, 1);
  display.print(" d/s");

  display.display();
}


void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Roboter - Ausrichtung mit QMC5883L Initialisierung...");

  Wire.begin(); // Wichtig: Vor qmc.init() aufrufen!

  // OLED Display initialisieren
  Serial.println("Initialisiere OLED Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("Fehler: SSD1306 Allokierung fehlgeschlagen!"));
    oledInitialized = false;
  } else {
    Serial.println("OLED Display erfolgreich initialisiert.");
    oledInitialized = true;
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.setRotation(2);
    Serial.println("Display-Rotation auf 180 Grad gesetzt.");
    display.println("OLED OK"); display.display(); delay(1000);
  }

  // QMC5883L Kompass initialisieren
  Serial.println("Initialisiere QMC5883L Kompass (mit QMC5883LCompass Lib)...");
  qmc.init(); // Initialisiert den Sensor (erwartet Adresse 0x0D)
  // Die qmc.init() der MPrograms Bibliothek gibt keinen direkten Fehlercode zurück.
  // Wir verlassen uns darauf, dass der I2C-Scanner 0x0D anzeigt.
  // Später können wir qmc.read() und qmc.getAzimuth() verwenden.
  Serial.println("QMC5883L init() aufgerufen. Sensor sollte an 0x0D sein.");
  Serial.println("KALIBRIERUNG ERFORDERLICH für genaue Werte! Siehe Bibliotheks-Beispiele.");
  // Beispiel: qmc.setCalibrationValues(-1803, 2087, -2159, 1.010, 0.982, 1.009); // Beispielwerte!

  if (oledInitialized) {
      display.clearDisplay(); display.setCursor(0,10); // Etwas Platz lassen
      display.println("QMC5883L init.");
      display.println("KALIBRIEREN!");
      display.display();
      delay(1000);
  }


  // MPU-6050 Gyroskop initialisieren
  Serial.println("Initialisiere MPU6050 Gyroskop/Beschleunigungssensor...");
  if (!mpu.begin()) {
    Serial.println("Fehler: MPU6050 nicht gefunden!");
     if (oledInitialized) {
        display.clearDisplay(); display.setCursor(0,0);
        display.println("MPU6050 FEHLER!"); display.println("Verkabelung pruefen.");
        display.display();
    }
    while (1) { delay(100); }
  }
  Serial.println("MPU6050 erfolgreich initialisiert.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Motor PWM Kanäle initialisieren
  Serial.println("Initialisiere Motor PWM Kanäle...");
  ledcSetup(CH_M_L_IN1, PWM_FREQ_MOT, PWM_RES_MOT); ledcSetup(CH_M_L_IN2, PWM_FREQ_MOT, PWM_RES_MOT);
  ledcSetup(CH_M_R_IN3, PWM_FREQ_MOT, PWM_RES_MOT); ledcSetup(CH_M_R_IN4, PWM_FREQ_MOT, PWM_RES_MOT);
  Serial.println("Verbinde Motor Pins mit PWM Kanälen...");
  ledcAttachPin(PIN_M_L_IN1, CH_M_L_IN1); ledcAttachPin(PIN_M_L_IN2, CH_M_L_IN2);
  ledcAttachPin(PIN_M_R_IN3, CH_M_R_IN3); ledcAttachPin(PIN_M_R_IN4, CH_M_R_IN4);
  stopRobot();
  Serial.println("Motor PWM Initialisierung abgeschlossen.");

  // Bluepad32 initialisieren
  Serial.println("Initialisiere Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  currentState = IDLE;
  targetHeadingDegrees = getTargetAngleFromCardinal(targetCardinalDirection);
  Serial.print("Ziel-Himmelsrichtung: "); Serial.print(targetCardinalDirection);
  Serial.print(" ("); Serial.print(targetHeadingDegrees); Serial.println(" Grad)");
  Serial.println("Setup abgeschlossen. Warte auf Controller-Verbindung...");
  
  if (oledInitialized) updateOLED();
}

void loop() {
  bool dataUpdated = BP32.update();

  if (millis() - lastSensorReadTime > 50) {
    // QMC5883L Heading lesen
    qmc.read(); // Neue Daten vom Sensor lesen
    currentHeadingQMC = qmc.getAzimuth(); // Azimut in Grad bekommen (0-359.99)
    // Die getAzimuth() Funktion der MPrograms Bibliothek berücksichtigt oft keine manuelle Deklination,
    // oder hat eine setDeclination() Methode. Für reine magnetische Ausrichtung ist das ok.

    // MPU6050 Gyro Daten holen
    sensors_event_t event_accel, event_gyro, event_temp; // MPU6050 braucht dies weiterhin
    mpu.getEvent(&event_accel, &event_gyro, &event_temp);
    currentGyroZ_degS = event_gyro.gyro.z * (180.0f / M_PI);
    
    lastSensorReadTime = millis();
    // Debugging-Ausgabe für Kompass (kannst du aktivieren, wenn nötig)
    /*
    Serial.print("QMC Roh-X: "); Serial.print(qmc.getX());
    Serial.print(" Y: "); Serial.print(qmc.getY());
    Serial.print(" Z: "); Serial.println(qmc.getZ());
    Serial.print("Aktueller QMC Heading: "); Serial.println(currentHeadingQMC, 1);
    */
  }

  if (myController && myController->isConnected() && dataUpdated) {
    if (myController->x() && currentState == IDLE) {
      targetHeadingDegrees = getTargetAngleFromCardinal(targetCardinalDirection);
      Serial.print("Quadrat-Taste. Ausrichtung auf: "); Serial.print(targetCardinalDirection);
      Serial.print(" ("); Serial.print(targetHeadingDegrees, 1); Serial.println(" G)");
      Serial.print("Aktuelle Ausrichtung: "); Serial.print(currentHeadingQMC, 1); Serial.println(" G");
      currentState = ROTATING_TO_TARGET;
    }
    if (myController->a()) {
      if (currentState == ROTATING_TO_TARGET) Serial.println("X-Taste: Drehung abgebrochen.");
      else Serial.println("X-Taste: Roboter gestoppt.");
      stopRobot(); currentState = IDLE;
    }
  } else if (myController == nullptr && currentState == ROTATING_TO_TARGET) {
      Serial.println("Controller getrennt. Stoppe Roboter.");
      stopRobot(); currentState = IDLE;
  }

  if (currentState == ROTATING_TO_TARGET) {
    float angleError = normalizeAngle180(targetHeadingDegrees - currentHeadingQMC);
    if (abs(angleError) <= DEADZONE_DEGREES) {
      Serial.println("Ziel erreicht. Stoppe Roboter.");
      stopRobot(); currentState = IDLE;
    } else {
      bool rotateCCW = (angleError > 0);
      rotateRobot(rotateCCW, ROTATION_PWM_SPEED);
    }
  }

  if (oledInitialized && (millis() - lastOledUpdateTime > 100)) {
    updateOLED();
    lastOledUpdateTime = millis();
  }
  
  delay(10);
}
