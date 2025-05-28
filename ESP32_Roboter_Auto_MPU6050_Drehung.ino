/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2025-05-27
 * Version: 1.9
 * Description:
 *
 * Dieses ESP32-Programm steuert ein Roboterfahrzeug mittels eines
 * Bluepad32-kompatiblen Game-Controllers. Es nutzt einen L298N
 * Motortreiber (angesteuert über ESP32 LEDC PWM) und einen MPU6050
 * Gyrosensor zur präzisen Drehung um die Z-Achse. Bei Knopfdruck
 * (X-Taste des Controllers) führt der Roboter eine Drehung um einen
 * vordefinierten Winkel (hier ca. 45 Grad) aus. Der aktuelle
 * Drehwinkel und Status werden auf einem SSD1306 I2C OLED-Display
 * angezeigt, das für Über-Kopf-Montage um 180 Grad gedreht ist.
 * Die A-Taste des Controllers stoppt die aktuelle Bewegung oder
 * bricht die laufende Drehung ab. Verwendete Bibliotheken sind
 * Bluepad32, Adafruit MPU6050, Adafruit GFX und Adafruit SSD1306.
 *
 * Hardware-Setup:
 * - ESP32 Mikrocontroller
 * - Bluepad32-kompatibler Game-Controller (z.B. PS4, PS5)
 * - L298N Motortreiber Modul
 * - Linker Motor: IN1 (GPIO 13), IN2 (GPIO 12)
 * - Rechter Motor: IN3 (GPIO 27), IN4 (GPIO 33)
 * - MPU6050 Gyroskop/Beschleunigungssensor (I2C)
 * - SSD1306 OLED Display (128x64, I2C, Adresse 0x3C)
 * - I2C SDA: Standard ESP32 I2C Pin (z.B. GPIO 21)
 * - I2C SCL: Standard ESP32 I2C Pin (z.B. GPIO 22)
 *
 * Controller-Tastenbelegung (Beispiel):
 * - X-Taste (Square): Startet die 45-Grad-Drehung (gegen den Uhrzeigersinn)
 * - A-Taste (Cross): Stoppt alle Motoren / Bricht Drehung ab
 *
 ****************************************************/
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Bluepad32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Einstellungen
#define SCREEN_WIDTH 128 // OLED Display Breite in Pixeln
#define SCREEN_HEIGHT 64 // OLED Display Höhe in Pixeln
#define OLED_RESET -1    // Reset Pin (oder -1, wenn nicht verwendet/geteilt)
#define OLED_ADDRESS 0x3C // Typische I2C Adresse für SSD1306
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledInitialized = false; 

// ----------------------------
// L298N Motor Driver INx Pin Definitions
// ----------------------------
#define PIN_M_L_IN1 13  
#define PIN_M_L_IN2 12  
#define PIN_M_R_IN3 27  
#define PIN_M_R_IN4 33  

// ----------------------------
// ESP32 LEDC PWM Config for INx Pins
// ----------------------------
#define CH_M_L_IN1 0
#define CH_M_L_IN2 1
#define CH_M_R_IN3 2
#define CH_M_R_IN4 3

#define PWM_FREQ_MOT 5000
#define PWM_RES_MOT 8
const int PWM_MAX_MOT = (1 << PWM_RES_MOT) - 1;

// ----------------------------
// MPU-6050 Sensor Objekt
// ----------------------------
Adafruit_MPU6050 mpu;

// ----------------------------
// Bluepad32 Controller Objekt
// ----------------------------
ControllerPtr myController = nullptr;

// ----------------------------
// Rotationslogik Variablen
// ----------------------------
float currentAngleZ = 0.0f;
unsigned long lastTimeIntegration = 0;
// unsigned long lastOledUpdateTime = 0; // Nicht mehr benötigt für den direkten Aufruf
bool isRotating = false;

const int ROTATION_PWM_SPEED = 150;
const float TARGET_ROTATION_ANGLE = 45.0f;
const float ROTATION_DEADZONE_MIN_ANGLE = TARGET_ROTATION_ANGLE - 5.0f; 

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
        isRotating = false;
    }
}

// --- Motorsteuerungsfunktionen ---
void setMotorPower(int motorNum, int power) {
    uint8_t pinA_pwm_channel, pinB_pwm_channel;
    if (motorNum == 0) { 
        pinA_pwm_channel = CH_M_L_IN1; pinB_pwm_channel = CH_M_L_IN2;
    } else { 
        pinA_pwm_channel = CH_M_R_IN3; pinB_pwm_channel = CH_M_R_IN4;
    }
    power = constrain(power, -PWM_MAX_MOT, PWM_MAX_MOT);
    if (power > 0) {
        ledcWrite(pinA_pwm_channel, power); ledcWrite(pinB_pwm_channel, 0);
    } else if (power < 0) {
        ledcWrite(pinA_pwm_channel, 0); ledcWrite(pinB_pwm_channel, abs(power));
    } else {
        ledcWrite(pinA_pwm_channel, 0); ledcWrite(pinB_pwm_channel, 0);
    }
}

void stopRobot() {
    Serial.println("DEBUG: stopRobot() aufgerufen.");
    setMotorPower(0, 0); setMotorPower(1, 0);
}

void rotateRobot(bool counterClockwise, int pwmSpeed) {
    Serial.print("DEBUG: rotateRobot() CCW: "); Serial.print(counterClockwise); Serial.print(", Speed: "); Serial.println(pwmSpeed);
    if (counterClockwise) {
        setMotorPower(0, -pwmSpeed); setMotorPower(1, pwmSpeed);  
    } else {
        setMotorPower(0, pwmSpeed); setMotorPower(1, -pwmSpeed); 
    }
}

// --- OLED Display Funktion ---
void updateOLED() {
    if (!oledInitialized) return; 

    display.clearDisplay(); // Displaypuffer löschen
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Zeile 1: Controller-Status
    display.setCursor(0, 0);
    if (myController && myController->isConnected()) {
        display.print("Ctrl: Verbunden");
    } else {
        display.print("Ctrl: Getrennt");
    }

    // Zeile 2: Aktueller Winkel
    display.setCursor(0, 10);
    display.print("Winkel: ");
    display.print(currentAngleZ, 1); 
    display.print(" G");

    // Zeile 3: Rotationsstatus
    display.setCursor(0, 20);
    if (isRotating) {
        display.print("Status: Dreht...");
    } else {
        display.print("Status: Bereit");
    }
    
    // Zeile 4: Zielinfo (optional, aber hilfreich)
    display.setCursor(0, 30);
    display.print("Ziel: ");
    display.print(TARGET_ROTATION_ANGLE, 0);
    display.print(" G (bei ");
    if (TARGET_ROTATION_ANGLE > 0) {
        display.print (">="); display.print(ROTATION_DEADZONE_MIN_ANGLE,0);
    } else { 
        display.print ("<="); display.print(TARGET_ROTATION_ANGLE + 5.0f, 0); 
    }
    display.print("G)");

    display.display(); // Inhalt des Puffers an das Display senden
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Roboter - 45 Grad Drehung Initialisierung...");

    Wire.begin(); 

    Serial.println("Initialisiere OLED Display...");
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) { 
        Serial.println(F("Fehler: SSD1306 Allokierung fehlgeschlagen!"));
        oledInitialized = false;
    } else {
        Serial.println("OLED Display erfolgreich initialisiert.");
        oledInitialized = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        // WICHTIG: Display-Ausrichtung um 180 Grad drehen (für Über-Kopf-Montage)
        // Rotation: 0 = 0 Grad, 1 = 90 Grad, 2 = 180 Grad, 3 = 270 Grad
        display.setRotation(2); 
        Serial.println("Display-Rotation auf 180 Grad gesetzt.");
        display.println("OLED OK"); // Testnachricht
        display.display();
        delay(1000); // Diese Nachricht 1 Sekunde anzeigen lassen
    }

    Serial.println("Initialisiere MPU6050...");
    if (!mpu.begin()) {
        Serial.println("Fehler: MPU6050 nicht gefunden! Verkabelung prüfen.");
        if (oledInitialized) { 
            display.clearDisplay(); display.setCursor(0,0); display.setTextSize(1);
            display.println("MPU6050 FEHLER!"); display.println("Verkabelung"); display.println("pruefen.");
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

    Serial.println("Setup abgeschlossen. Warte auf Controller-Verbindung...");
    lastTimeIntegration = micros();
    if (oledInitialized) updateOLED(); // Initiales Update des OLEDs nach Setup
}

void loop() {
    bool dataUpdated = BP32.update(); // Wichtig für Controller-Daten

    sensors_event_t a_event, g_event, temp_event;
    mpu.getEvent(&a_event, &g_event, &temp_event); // Sensor Daten holen

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTimeIntegration) / 1000000.0f;
    lastTimeIntegration = currentTime;

    float gyroZ_degS = g_event.gyro.z * (180.0f / M_PI); 

    if (myController && myController->isConnected() && dataUpdated) {
        if (myController->x() && !isRotating) { 
            Serial.println("'Square' (controller->x()) gedrückt: Starte 45° Drehung CCW.");
            currentAngleZ = 0.0f;
            lastTimeIntegration = micros(); // Zeitmessung für Integration neu starten
            isRotating = true;
            rotateRobot(true, ROTATION_PWM_SPEED);
        }

        if (myController->a()) { 
            if (isRotating) {
                Serial.println("'X' (controller->a()) gedrückt: Drehung abgebrochen.");
            } else {
                Serial.println("'X' (controller->a()) gedrückt: Roboter gestoppt.");
            }
            stopRobot();
            isRotating = false;
        }
    } else if (myController == nullptr && isRotating) {
        Serial.println("Controller während Drehung getrennt. Stoppe Roboter.");
        stopRobot();
        isRotating = false;
    }

    if (isRotating) {
        // WICHTIG: Passen Sie das Vorzeichen hier an, wenn der Winkel in die "falsche" Richtung zählt!
        // Wenn Ihre Konsole/OLED negative Winkel bei CCW-Drehung anzeigt, und Sie möchten, dass
        // currentAngleZ positiv wird für CCW (um das positive Ziel 45° zu erreichen):
        // Option 1: currentAngleZ -= gyroZ_degS * deltaTime; 
        // Option 2: float gyroZ_degS = -(g_event.gyro.z * (180.0f / M_PI));
        currentAngleZ += gyroZ_degS * deltaTime; 

        // Serielle Ausgabe für Debugging beibehalten
        Serial.print("Drehung aktiv - Aktueller Winkel: "); Serial.print(currentAngleZ, 2); Serial.println(" Grad");

        if (currentAngleZ >= ROTATION_DEADZONE_MIN_ANGLE) { 
             Serial.print("Zieldrehwinkel erreicht. Stoppe bei: "); Serial.print(currentAngleZ, 2); Serial.println(" Grad");
             stopRobot();
             isRotating = false;
        }
    }

    // OLED Display in jedem Loop-Durchlauf aktualisieren
    if (oledInitialized) {
        updateOLED();
    }
    
    delay(1); // Kurze Pause für Stabilität und um anderen Prozessen Zeit zu geben
}
