/***************************************************
 * Autor: Ingmar Stapel
 * Webseite: www.custom-build-robots.com
 * Datu,: 2025-05-24
 * Version: 1.0 
 *
 * ESP32 Linienfolger-Roboter mit PS5 Controller und OLED Display
 *
 * Kombiniert Bluepad32 für PS5-Controller-Eingabe, L298N
 * Motorsteuerung (PWM an INx), KY-033 Liniensensoren und
 * Adafruit SSD1306 OLED-Display.
 *
 * Funktionen:
 * - Autonomes Linienfolgen.
 * - PS5-Controller:
 * - "Options"-Taste: Startet Linienfolgemodus. (Aktuell Quadrat-Taste)
 * - "Kreuz" (X)-Taste: Not-Aus, zurück in Standby.
 * - OLED-Display: Zeigt Status (Controller, Modus, Sensoren).
 * - Motorsteuerung: PWM an L298N INx-Pins
 * (ENA/ENB auf HIGH gejumpert).
 * - Sensoren: Drei digitale KY-033 Liniensensoren.
 * - Suchfunktion bei Linienverlust.
 *
 ********************************************************************/

// ----------------------------
// Bibliotheken
// ----------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bluepad32.h>

// ----------------------------
// Roboter Konfiguration
// ----------------------------
const String ROBOT_NAME = "ESP32 LineBot";

// ----------------------------
// L298N Motor-Treiber INx Pin Definitionen
// ----------------------------
#define PIN_M_L_IN1 13
#define PIN_M_L_IN2 12
#define PIN_M_R_IN3 27
#define PIN_M_R_IN4 33

// ----------------------------
// ESP32 LEDC PWM Konfiguration
// ----------------------------
#define CH_M_L_IN1 0
#define CH_M_L_IN2 1
#define CH_M_R_IN3 2
#define CH_M_R_IN4 3
#define PWM_FREQ_MOT 5000
#define PWM_RES_MOT 8
const int PWM_MAX_MOT = (1 << PWM_RES_MOT) - 1;

// ----------------------------
// Linienfolger Sensor Pin Definitionen
// ----------------------------
const int SENSOR_L_PIN = 34;
const int SENSOR_M_PIN = 2;
const int SENSOR_R_PIN = 35;

// ----------------------------
// OLED Display Konfiguration
// ----------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long lastOledUpdate = 0;
const long OLED_UPDATE_INTERVAL = 250;

// ----------------------------
// Bluepad32 Controller Verwaltung
// ----------------------------
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool controllerConnected = false;
unsigned long lastControllerActivity = 0;
const unsigned long CONTROLLER_TIMEOUT_MS = 5000;

// ----------------------------
// Roboter Zustandsmaschine (State Machine)
// ----------------------------
enum RobotState
{
    STANDBY,
    LINE_FOLLOW_READY,
    LINE_FOLLOWING,
    LINE_LOST_SEARCHING // <-- NEW STATE
};
RobotState currentRobotState = STANDBY;

String getRobotStateName(RobotState state) {
    switch (state) {
        case STANDBY:             return "Standby";
        case LINE_FOLLOW_READY:   return "LF Bereit";
        case LINE_FOLLOWING:      return "LF Aktiv";
        case LINE_LOST_SEARCHING: return "LF Suche"; // <-- NEW STATE NAME
        default:                  return "Unbekannt";
    }
}

// ----------------------------
// Linienfolger Geschwindigkeiten (0-100%)
// ----------------------------
const int NORM_GESCHW = 30;
const int KORR_GESCHW_SCHNELL = 40;
const int KORR_GESCHW_LANGSAM = 20;
const int TURN_GESCHW = 35; // Optional, not explicitly used by current line follow

// ----------------------------
// Search Maneuver Variables <-- NEW
// ----------------------------
unsigned long searchManeuverStartTime = 0;
const unsigned long MAX_SEARCH_DURATION_MS = 2500; // Try searching for up to 2.5 seconds
const int SEARCH_TURN_SPEED = 35; // Speed for turning during search

// ----------------------------
// Sensor Zustände
// ----------------------------
bool sL_val = false;
bool sM_val = false;
bool sR_val = false;

// Forward-Deklarationen
void updateOLED();
void stopMotors();
void readSensorStates(); // Ensure this is declared if defined later
void executeLineFollowingAlgorithm(); // Ensure this is declared

// --------------------------------------------------------------------
// MOTORSTEUERUNG
// --------------------------------------------------------------------
void setMotor(int side, int speedPct)
{
    uint8_t ch_A, ch_B;
    if (side == 0) { ch_A = CH_M_L_IN1; ch_B = CH_M_L_IN2; }
    else { ch_A = CH_M_R_IN3; ch_B = CH_M_R_IN4; }

    int dutyCycle = map(abs(speedPct), 0, 100, 0, PWM_MAX_MOT);
    dutyCycle = constrain(dutyCycle, 0, PWM_MAX_MOT);

    if (speedPct > 0) { ledcWrite(ch_A, dutyCycle); ledcWrite(ch_B, 0); }
    else if (speedPct < 0) { ledcWrite(ch_A, 0); ledcWrite(ch_B, dutyCycle); }
    else { ledcWrite(ch_A, 0); ledcWrite(ch_B, 0); }
}

void stopMotors()
{
    setMotor(0, 0);
    setMotor(1, 0);
    Serial.println("Motoren GESTOPPT");
}

// ----------------------------
// Bluepad32 Verbindungs-Callbacks
// ----------------------------
void onCtrlConnect(ControllerPtr ctl)
{
    bool slotFound = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller verbunden, idx=%d\n", i);
            myControllers[i] = ctl;
            slotFound = true;
            break;
        }
    }
    if (!slotFound) Serial.println("Controller verbunden, aber kein freier Slot");
    controllerConnected = true;
    lastControllerActivity = millis();
    updateOLED(); // Update OLED after connection status change
}

void onCtrlDisconnect(ControllerPtr ctl)
{
    bool ctrlFound = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller getrennt, idx=%d\n", i);
            myControllers[i] = nullptr;
            ctrlFound = true;
            break;
        }
    }
    if (!ctrlFound) Serial.println("Ctrl getrennt, nicht in Liste gefunden");

    controllerConnected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            controllerConnected = true;
            break;
        }
    }

    if (!controllerConnected) {
        Serial.println("Alle Controller getrennt. Motoren stoppen und Rückkehr in Standby.");
        stopMotors();
        currentRobotState = STANDBY;
    }
    updateOLED(); // Update OLED after connection status change
}

// ----------------------------
// Sensor Lese-Funktion (Original, falls benötigt)
// ----------------------------
void readSensorStates_old()
{
    // KY-033: LOW = Linie (dunkel),
    // HIGH = Keine Linie / Boden (hell)
    sL_val = (digitalRead(SENSOR_L_PIN) == LOW);
    sM_val = (digitalRead(SENSOR_M_PIN) == LOW);
    sR_val = (digitalRead(SENSOR_R_PIN) == LOW);
}

// ----------------------------
// Sensor Lese-Funktion (Aktuell verwendet)
// ----------------------------
void readSensorStates()
{
    // Annahme basierend auf vorheriger Fehlerbehebung:
    // HIGH = Dunkle Linie (oder was auch immer als "Linie" definiert ist)
    // LOW  = Heller Untergrund (keine Linie)
    sL_val = (digitalRead(SENSOR_L_PIN) == HIGH);
    sM_val = (digitalRead(SENSOR_M_PIN) == HIGH);
    sR_val = (digitalRead(SENSOR_R_PIN) == HIGH);
}

// ----------------------------
// OLED Display Update Funktion
// ----------------------------
void updateOLED()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(ROBOT_NAME);
    display.print("Ctrl: ");
    display.println(controllerConnected ? "Verbunden" : "Getrennt");
    display.print("Modus: ");
    display.println(getRobotStateName(currentRobotState));
    display.print("Sens: L["); display.print(sL_val ? "1" : "0");
    display.print("] M["); display.print(sM_val ? "1" : "0");
    display.print("] R["); display.print(sR_val ? "1" : "0");
    display.println("]");
    bool lineVisible = sL_val || sM_val || sR_val;
    if (currentRobotState == LINE_FOLLOWING ||
        currentRobotState == LINE_FOLLOW_READY ||
        currentRobotState == LINE_LOST_SEARCHING) { // Include searching state for line visibility
        display.print("Linie: ");
        display.println(lineVisible ? "Ja" : "Nein");
    }
    display.display();
}

// ----------------------------
// Controller Eingabe Verarbeitung
// ----------------------------
void processControllerInputs()
{
    ControllerPtr ctl = nullptr;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            ctl = myControllers[i];
            break;
        }
    }

    if (ctl && ctl->hasData()) {
        lastControllerActivity = millis();
        if (ctl->x()) { // Quadrat-Taste
            if (currentRobotState == STANDBY) {
                currentRobotState = LINE_FOLLOW_READY;
                Serial.println("Modus -> LF Bereit (Quadrat-Taste, wartet auf Linie)");
                stopMotors();
                delay(200);
            }
        }
        if (ctl->a()) { // Kreuz-Taste
            Serial.println("NOT-AUS gedrückt (via Kreuz-Taste)!");
            stopMotors();
            currentRobotState = STANDBY;
            Serial.println("Modus -> STANDBY");
            delay(200);
        }
    }
}

// ----------------------------
// Linienfolge-Algorithmus
// ----------------------------
void executeLineFollowingAlgorithm()
{
    readSensorStates(); // Sensorwerte zuerst lesen

    // Linie verloren: Wenn alle Sensoren keine Linie sehen
    if (!sL_val && !sM_val && !sR_val) {
        Serial.println("Linie verloren! Wechsle zu Suchmodus (LINE_LOST_SEARCHING).");
        currentRobotState = LINE_LOST_SEARCHING; // Zustand wechseln
        searchManeuverStartTime = millis();     // Startzeit für Suche merken
        stopMotors(); // Kurz anhalten, bevor Suchmanöver im neuen Zustand beginnt
        return;       // Wichtig: Funktion hier verlassen, loop() kümmert sich um neuen Zustand
    }

    // Wenn Linie NICHT verloren ist (mindestens ein Sensor sieht die Linie):
    // Normale Linienfolge-Logik
    if (sM_val) {
        Serial.println("Linie: Mitte -> Geradeaus");
        setMotor(0, NORM_GESCHW);
        setMotor(1, NORM_GESCHW);
    }
    else if (sL_val && !sR_val) {
        Serial.println("Linie: Links -> Nach Links");
        setMotor(0, KORR_GESCHW_LANGSAM);
        setMotor(1, KORR_GESCHW_SCHNELL);
    }
    else if (sR_val && !sL_val) {
        Serial.println("Linie: Rechts -> Nach Rechts");
        setMotor(0, KORR_GESCHW_SCHNELL);
        setMotor(1, KORR_GESCHW_LANGSAM);
    }
    else {
        Serial.println("Linie: Undefinierter Sensorzustand -> Stopp");
        stopMotors();
    }
}

// ----------------------------
// SETUP FUNKTION
// ----------------------------
void setup()
{
    Serial.begin(115200);
    Serial.println("\nInitialisiere " + ROBOT_NAME + "...");

    Wire.begin();
    Serial.println("I2C Bus initialisiert.");

    Serial.println("Initialisiere SSD1306 OLED Display...");
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 Allokierung/Display nicht gefunden!"));
        while (1) delay(10);
    }
    display.setRotation(2);
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.println(ROBOT_NAME); display.println("Initialisiere...");
    display.display();
    Serial.println("OLED Display initialisiert.");

    Serial.println("Initialisiere Motor PWM Kanäle...");
    ledcSetup(CH_M_L_IN1, PWM_FREQ_MOT, PWM_RES_MOT);
    ledcSetup(CH_M_L_IN2, PWM_FREQ_MOT, PWM_RES_MOT);
    ledcSetup(CH_M_R_IN3, PWM_FREQ_MOT, PWM_RES_MOT);
    ledcSetup(CH_M_R_IN4, PWM_FREQ_MOT, PWM_RES_MOT);
    ledcAttachPin(PIN_M_L_IN1, CH_M_L_IN1);
    ledcAttachPin(PIN_M_L_IN2, CH_M_L_IN2);
    ledcAttachPin(PIN_M_R_IN3, CH_M_R_IN3);
    ledcAttachPin(PIN_M_R_IN4, CH_M_R_IN4);
    Serial.println("Motor PWM bereit.");
    stopMotors();

    Serial.println("Initialisiere Liniensensoren...");
    pinMode(SENSOR_L_PIN, INPUT);
    pinMode(SENSOR_M_PIN, INPUT);
    pinMode(SENSOR_R_PIN, INPUT);
    Serial.println("Sensor-Pins als INPUT konfiguriert.");

    Serial.println("Initialisiere Bluepad32...");
    BP32.setup(&onCtrlConnect, &onCtrlDisconnect);
    BP32.enableVirtualDevice(false);
    // BP32.forgetBluetoothKeys(); // Einkommentieren, um Pairing zu erzwingen

    currentRobotState = STANDBY;
    Serial.println("Setup abgeschlossen. Roboter im STANDBY Modus.");
    updateOLED(); // Initialen Status anzeigen
}

// ----------------------------
// HAUPTSCHLEIFE (LOOP)
// ----------------------------
void loop()
{
    if (BP32.update()) { /* Bluetooth-Events verarbeitet */ }

    if (controllerConnected) {
        processControllerInputs();
        unsigned long timeDiff = millis() - lastControllerActivity;
        if (timeDiff > CONTROLLER_TIMEOUT_MS) {
            Serial.println("Controller Timeout. Stopp & Standby.");
            stopMotors();
            currentRobotState = STANDBY;
            lastControllerActivity = millis(); // Verhindert sofortiges Neutriggern
        }
    }

    // Aktionen der Roboter-Zustandsmaschine
    switch (currentRobotState)
    {
    case STANDBY:
        readSensorStates(); // Nur Sensoren lesen für Anzeige, Motoren sind aus
        break;

    case LINE_FOLLOW_READY:
        readSensorStates();
        if (sM_val) { // Mittlerer Sensor erkennt Linie
            currentRobotState = LINE_FOLLOWING;
            Serial.println("Modus -> LF Aktiv (Mittlerer Sensor hat Linie erkannt)");
            // executeLineFollowingAlgorithm() wird im nächsten Schleifendurchlauf aufgerufen
        }
        break;

    case LINE_FOLLOWING:
        executeLineFollowingAlgorithm(); // Kann Zustand zu LINE_LOST_SEARCHING ändern
        break;

    case LINE_LOST_SEARCHING: // <-- NEUER ZUSTAND IM LOOP
        readSensorStates(); // Sensoren kontinuierlich prüfen

        if (sL_val || sM_val || sR_val) { // Linie während Suche wiedergefunden?
            Serial.println("Linie während Suche wiedergefunden! Zurück zu LF Aktiv.");
            currentRobotState = LINE_FOLLOWING;
            // Im nächsten Durchlauf wird executeLineFollowingAlgorithm() wieder normal ausgeführt
        }
        else if (millis() - searchManeuverStartTime > MAX_SEARCH_DURATION_MS) { // Suchzeit abgelaufen?
            Serial.println("Suche beendet (Timeout), Linie nicht gefunden. Stoppe und gehe zu STANDBY.");
            stopMotors();
            currentRobotState = STANDBY;
        }
        else { // Andernfalls Suchmanöver fortsetzen
            // Beispiel: Links drehen (auf der Stelle)
            // Serial.println("Suche aktiv: Drehe links..."); // Optional: kann Serial Monitor überfluten
            setMotor(0, -SEARCH_TURN_SPEED); // Linker Motor rückwärts
            setMotor(1, SEARCH_TURN_SPEED);  // Rechter Motor vorwärts
        }
        break;
    }

    // OLED-Display periodisch aktualisieren
    unsigned long currentTime = millis();
    if (currentTime - lastOledUpdate > OLED_UPDATE_INTERVAL) {
        // Sicherstellen, dass Sensorwerte für Anzeige aktuell sind, falls nicht im aktiven Folgen/Suchen
        // In LINE_FOLLOWING werden sie in executeLineFollowingAlgorithm gelesen.
        // In LINE_LOST_SEARCHING werden sie am Anfang des case gelesen.
        // In STANDBY und LINE_FOLLOW_READY werden sie auch am Anfang des case gelesen.
        // Daher ist der extra readSensorStates() hier ggf. nicht mehr zwingend nötig,
        // aber schadet auch nicht, wenn man sichergehen will.
        if (currentRobotState != LINE_FOLLOWING && currentRobotState != LINE_LOST_SEARCHING) {
             readSensorStates(); // Für STANDBY und LF_READY sicherstellen
        }
        updateOLED();
        lastOledUpdate = currentTime;
    }
    delay(10); // Kurze Verzögerung für Stabilität
}