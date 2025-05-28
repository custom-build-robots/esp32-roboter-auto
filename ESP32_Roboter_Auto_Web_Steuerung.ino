/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2025-05-27
 * Version: 1.9
 * Description:
 *
 * Dieses ESP32-Programm ermöglicht einem Roboterfahrzeug autonomes
 * Fahren mit Hindernisvermeidung, gesteuert über ein Webinterface.
 * Der ESP32 verbindet sich mit einem WLAN und startet einen Webserver,
 * über den der autonome Modus (Vorwärtsfahrt) per Webbrowser
 * gestartet und gestoppt werden kann. Zur Hinderniserkennung dient
 * ein HC-SR04 Ultraschallsensor. Bei Unterschreitung einer definierten
 * Distanz stoppt der Roboter, dreht sich um einen zufälligen Winkel
 * (45-90 Grad) in eine zufällige Richtung und setzt dann die Fahrt fort.
 * Ein MPU6050 Gyrosensor misst den Drehwinkel für das Ausweichmanöver.
 * Ein L298N Motortreiber (angesteuert via ESP32 LEDC PWM) steuert
 * die Motoren. Statusinformationen (WiFi-Status, IP-Adresse, Distanz,
 * Roboterzustand, Gyro-Daten) werden auf einem SSD1306 I2C OLED-
 * Display (180° gedreht) angezeigt.
 *
 * Hardware-Setup:
 * - ESP32 Mikrocontroller (mit WiFi)
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
 * - Netzwerk: WLAN-Zugangspunkt (Router) mit bekannter SSID/Passwort
 *
 * Web Interface:
 * - Erreichbar über die IP-Adresse des ESP32 im Browser.
 * - Buttons: "Start" (beginnt autonome Fahrt), "Stopp" (hält Roboter an).
 *
 * Hinweis: Die Logik für driveForward() und rotateRobot() ist im Code
 * an eine spezifische (möglicherweise invertierte) Motor-Grunddrehrichtung
 * angepasst. Bitte WLAN SSID und Passwort im Code eintragen!
 ****************************************************/
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cstdlib> // Für random()

// NEU: WiFi und Web Server Bibliotheken
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// --- WiFi Zugangsdaten ---
const char* ssid = "DEINE_WIFI_SSID";       // HIER DEINE WLAN SSID EINTRAGEN
const char* password = "DEIN_WIFI_PASSWORT"; // HIER DEIN WLAN PASSWORT EINTRAGEN

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
String oledWifiStatus = "WiFi Init..."; // Für OLED-Anzeige des WiFi-Status

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

// NEU: AsyncWebServer Objekt auf Port 80
AsyncWebServer server(80);

// Roboter Zustände
enum RobotState {
  IDLE,
  DRIVING_FORWARD,
  AVOIDING_OBSTACLE_ROTATING
};
RobotState currentState = IDLE;
String currentStateName = "IDLE"; // Für OLED-Anzeige

// Rotations- und Fahrlogik Variablen
float currentTurnProgressAngle = 0.0f;
float targetRelativeTurnAngle = 0.0f;
bool isTurningCW = false;

unsigned long lastTimeIntegration = 0;
unsigned long lastOledUpdateTime = 0;
unsigned long lastDistanceReadTime = 0;

const int FORWARD_PWM_SPEED = 100;
const int ROTATION_PWM_SPEED = 80;
const float TURN_DEADZONE_DEGREES = 5.0f;
const float OBSTACLE_DISTANCE_CM = 30.0f;
long currentDistanceCm = 0;

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

void driveForward(int pwmSpeed) {
  Serial.print("DEBUG: driveForward(), Speed: "); Serial.println(pwmSpeed);
  setMotorPower(0, -pwmSpeed);
  setMotorPower(1, -pwmSpeed);
}

void rotateRobot(bool counterClockwise, int pwmSpeed) {
  Serial.print("DEBUG: rotateRobot() CCW: "); Serial.print(counterClockwise);
  Serial.print(", Speed: "); Serial.println(pwmSpeed);
  if (counterClockwise) {
    setMotorPower(0, pwmSpeed);
    setMotorPower(1, -pwmSpeed);
  } else {
    setMotorPower(0, -pwmSpeed);
    setMotorPower(1, pwmSpeed);
  }
}

// --- HC-SR04 Distanzmessung ---
long readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 25000);
  if (duration == 0) {
    return 999;
  }
  long distance = duration / 58;
  return distance;
}

// --- OLED Display Funktion ---
void updateOLED() {
  if (!oledInitialized) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // WiFi Status (Zeilen 0-1)
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.println("WiFi: Verbunden");
    display.print(WiFi.localIP());
  } else {
    display.println(oledWifiStatus); // "WiFi Init...", "Verbinde WiFi...", oder "WiFi Fehler"
  }

  // Roboter Status (Zeilen 2-5)
  display.setCursor(0, 20);
  display.print("Modus: "); display.print(currentStateName);

  display.setCursor(0, 30);
  display.print("Dist: ");
  if (currentDistanceCm >= 999) {
    display.print("> "); display.print(OBSTACLE_DISTANCE_CM * 2);
  } else {
    display.print(currentDistanceCm);
  }
  display.print(" cm");

  display.setCursor(0, 40);
  if (currentState == AVOIDING_OBSTACLE_ROTATING) {
    display.print("Dreh: "); display.print(currentTurnProgressAngle, 0);
    display.print("/"); display.print(targetRelativeTurnAngle, 0); display.print(" G");
  } else if (currentState == DRIVING_FORWARD) {
    display.print("Ziel: Fahren");
  } else { // IDLE
    display.print("Ziel: Stop");
  }

  display.setCursor(0, 50);
  sensors_event_t a_event_oled, g_event_oled, temp_event_oled;
  mpu.getEvent(&a_event_oled, &g_event_oled, &temp_event_oled);
  float gyroZ_degS_oled = g_event_oled.gyro.z * (180.0f / M_PI);
  display.print("GZ: "); display.print(gyroZ_degS_oled, 1); // "GyroZ" gekürzt

  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Roboter - Web Interface Steuerung");

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
    display.setCursor(0, 0); display.setRotation(2); // Drehung beibehalten
    Serial.println("Display-Rotation auf 180 Grad gesetzt.");
    display.println("OLED OK");
    display.println(oledWifiStatus); // Zeigt "WiFi Init..."
    display.display(); delay(500);
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

  // --- WiFi Verbindung herstellen ---
  Serial.print("Verbinde mit WiFi: "); Serial.println(ssid);
  oledWifiStatus = "Verbinde WiFi...";
  if (oledInitialized) updateOLED();

  WiFi.mode(WIFI_STA); // ESP32 als WiFi-Station
  WiFi.begin(ssid, password);
  unsigned long wifiConnectStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiConnectStartTime < 15000)) { // Timeout 15 Sek.
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi verbunden!");
    Serial.print("IP Adresse: "); Serial.println(WiFi.localIP());
    oledWifiStatus = "WiFi Verbunden"; // Wird in updateOLED() verwendet oder direkt WiFi.status()
  } else {
    Serial.println("\nWiFi Verbindung fehlgeschlagen!");
    oledWifiStatus = "WiFi Fehler";
  }
  if (oledInitialized) updateOLED(); // Finalen WiFi-Status anzeigen

  // --- Web Server Routen definieren ---
  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = "<!DOCTYPE html><html><head><title>RoboCar Control</title>";
      html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
      html += "<style>body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; background-color: #f0f0f0; }";
      html += "h1 { color: #333; }";
      html += "button { background-color: #4CAF50; color: white; padding: 15px 32px;";
      html += "text-align: center; text-decoration: none; display: inline-block;";
      html += "font-size: 16px; margin: 10px; cursor: pointer; border-radius: 8px; border: none; width: 150px; box-shadow: 0 4px 8px 0 rgba(0,0,0,0.2); transition: 0.3s;}";
      html += "button:hover { box-shadow: 0 8px 16px 0 rgba(0,0,0,0.2); }";
      html += "#stopButton { background-color: #f44336; }";
      html += ".status { margin-top: 20px; font-size: 18px; color: #555; } </style></head><body>";
      html += "<h1>Roboter Auto Steuerung</h1>";
      html += "<button onclick=\"sendCommand('/start')\">Start</button>";
      html += "<button id='stopButton' onclick=\"sendCommand('/stop')\">Stopp</button>";
      html += "<div class='status' id='responseStatus'>Warte auf Befehl...</div>";
      html += "<script>function sendCommand(command) {";
      html += "document.getElementById('responseStatus').innerText = 'Sende: ' + command.substring(1) + '...';";
      html += "fetch(command).then(response => response.text()).then(data => {";
      html += "document.getElementById('responseStatus').innerText = 'Antwort: ' + data;";
      html += "}).catch(error => {";
      html += "document.getElementById('responseStatus').innerText = 'Fehler: ' + error;";
      html += "});}</script>";
      html += "</body></html>";
      request->send(200, "text/html", html);
    });

    server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
      if (currentState == IDLE) {
        Serial.println("Web Interface: Starte autonomen Modus.");
        currentState = DRIVING_FORWARD;
        currentStateName = "Fahren";
        driveForward(FORWARD_PWM_SPEED);
        request->send(200, "text/plain", "Autonomer Modus gestartet.");
      } else {
        request->send(200, "text/plain", "Bereits aktiv oder nicht im IDLE Modus.");
      }
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
      Serial.println("Web Interface: Stoppe Roboter.");
      stopRobot();
      currentState = IDLE;
      currentStateName = "IDLE";
      request->send(200, "text/plain", "Roboter gestoppt.");
    });

    server.begin(); // Webserver starten
    Serial.println("HTTP Server gestartet.");
    if (oledInitialized) {
        // updateOLED() wird im Loop regelmäßig aufgerufen und den Status anzeigen
    }
  } else {
    Serial.println("HTTP Server konnte nicht gestartet werden (keine WiFi Verbindung).");
     if (oledInitialized) {
        // updateOLED() zeigt "WiFi Fehler"
    }
  }

  currentState = IDLE;
  currentStateName = "IDLE";
  Serial.println("Setup abgeschlossen. Warte auf Web-Befehle...");
  lastTimeIntegration = micros();
  if (oledInitialized) updateOLED();
}

void loop() {
  // KEINE Controller-Verarbeitung mehr hier

  unsigned long currentTime = micros();

  // Zustandslogik des Roboters (bleibt größtenteils gleich)
  if (currentState == DRIVING_FORWARD) {
    if (millis() - lastDistanceReadTime > 100) {
      currentDistanceCm = readDistanceCm();
      Serial.print("Distanz: "); Serial.print(currentDistanceCm); Serial.println(" cm");
      lastDistanceReadTime = millis();
    }

    if (currentDistanceCm < OBSTACLE_DISTANCE_CM && currentDistanceCm > 0) {
      Serial.println("Hindernis erkannt!");
      stopRobot();
      targetRelativeTurnAngle = random(45, 91);
      isTurningCW = random(0, 2); // 0 für CW (im Uhrzeigersinn), 1 für CCW (gegen den Uhrzeigersinn)
      Serial.print("Starte Ausweichdrehung: "); Serial.print(targetRelativeTurnAngle);
      Serial.print(" Grad "); Serial.println(isTurningCW ? "CW" : "CCW");

      currentTurnProgressAngle = 0.0f;
      lastTimeIntegration = currentTime;
      currentState = AVOIDING_OBSTACLE_ROTATING;
      currentStateName = "Ausweichen";
      rotateRobot(!isTurningCW, ROTATION_PWM_SPEED); // rotateRobot(true) ist CCW
    }
    // Kein 'else' zum erneuten Starten von driveForward nötig,
    // da der Zustand DRIVING_FORWARD aktiv bleibt bis ein Hindernis kommt oder Stopp gedrückt wird.
  }
  else if (currentState == AVOIDING_OBSTACLE_ROTATING) {
    sensors_event_t a_event, g_event, temp_event;
    mpu.getEvent(&a_event, &g_event, &temp_event);
    float deltaTime = (currentTime - lastTimeIntegration) / 1000000.0f;
    lastTimeIntegration = currentTime;
    float gyroZ_radS = g_event.gyro.z;
    currentTurnProgressAngle += abs(gyroZ_radS * (180.0f / M_PI)) * deltaTime;

    Serial.print("Drehung Fortschritt: "); Serial.print(currentTurnProgressAngle, 1);
    Serial.print("/"); Serial.print(targetRelativeTurnAngle, 1); Serial.println(" Grad");

    if (currentTurnProgressAngle >= (targetRelativeTurnAngle - TURN_DEADZONE_DEGREES)) {
      Serial.println("Ausweichdrehung beendet.");
      stopRobot();
      currentState = DRIVING_FORWARD;
      currentStateName = "Fahren";
      delay(100); // Kurze Pause bevor es weitergeht
      driveForward(FORWARD_PWM_SPEED);
    }
  }

  // OLED Display aktualisieren (nicht in jeder Schleife, um Performance zu sparen)
  if (oledInitialized && (millis() - lastOledUpdateTime > 250)) { // z.B. alle 250ms
    updateOLED();
    lastOledUpdateTime = millis();
  }

  delay(10); // Kurze Pause für Stabilität
}
