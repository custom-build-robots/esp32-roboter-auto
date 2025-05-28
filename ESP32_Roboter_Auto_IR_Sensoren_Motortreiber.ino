/***************************************************
 * Autor: Ingmar Stapel
 * Webseite: www.custom-build-robots.com
 * Datu,: 2025-05-23
 * Version: 1.0 
 *
 * Beschreibung:
 * ESP32 Linienfolger-Roboter mit PS5 Controller und OLED Display
 *
 * Kombiniert Bluepad32 für PS5-Controller-Eingabe, L298N
 * Motorsteuerung (PWM an INx), KY-033 Liniensensoren und
 * Adafruit SSD1306 OLED-Display.
 *
 * Funktionen:
 * - Autonomes Linienfolgen.
 * - PS5-Controller:
 * - "Options"-Taste: Startet Linienfolgemodus.
 * - "Kreuz" (X)-Taste: Not-Aus, zurück in Standby.
 * - OLED-Display: Zeigt Status (Controller, Modus, Sensoren).
 * - Motorsteuerung: PWM an L298N INx-Pins
 * (ENA/ENB auf HIGH gejumpert).
 * - Sensoren: Drei digitale KY-033 Liniensensoren.
 *
 * Basiert auf Konzepten von:
 * - Ingmar Stapel (custom-build-robots.com) für Bluepad32 &
 * L298N PWM INx.
 * - Allgemeinen KY-033 und Adafruit_SSD1306 Beispielen.
 ********************************************************************/

// ----------------------------
// Bibliotheken
// ----------------------------
// Für I2C-Kommunikation (OLED Display)
#include <Wire.h>
// Basis-Grafikbibliothek für das Display
#include <Adafruit_GFX.h>
// Für das SSD1306 OLED Display
#include <Adafruit_SSD1306.h>
// Bluepad32-Bibliothek einbinden
#include <Bluepad32.h>

// ----------------------------
// Roboter Konfiguration
// ----------------------------
const String ROBOT_NAME = "ESP32 LineBot";

// ----------------------------
// L298N Motor-Treiber INx Pin Definitionen
// (ENA/ENB extern auf HIGH gejumpert)
// ----------------------------
// L298N IN1 (Linker Motor)
#define PIN_M_L_IN1 13
// L298N IN2 (Linker Motor)
#define PIN_M_L_IN2 12
// L298N IN3 (Rechter Motor)
#define PIN_M_R_IN3 27
// L298N IN4 (Rechter Motor)
#define PIN_M_R_IN4 33

// ----------------------------
// ESP32 LEDC PWM Konfiguration für INx Pins
// ----------------------------
#define CH_M_L_IN1 0
#define CH_M_L_IN2 1
#define CH_M_R_IN3 2
#define CH_M_R_IN4 3

// Motor PWM Frequenz (Hz)
#define PWM_FREQ_MOT 5000
// Motor PWM Auflösung (Bits)
#define PWM_RES_MOT 8
// Max. PWM-Wert (z.B. 255 für 8-Bit Auflösung)
const int PWM_MAX_MOT = (1 << PWM_RES_MOT) - 1;

// ----------------------------
// Linienfolger Sensor Pin Definitionen
// KY-033: LOW = Linie (dunkel),
// HIGH = Keine Linie / Boden (hell)
// ----------------------------
// Linker Sensor
const int SENSOR_L_PIN = 34;
// Mittlerer Sensor
const int SENSOR_M_PIN = 2;
// Rechter Sensor
const int SENSOR_R_PIN = 35;

// ----------------------------
// OLED Display Konfiguration (Adafruit SSD1306)
// ----------------------------
// OLED Displaybreite in Pixel
#define SCREEN_WIDTH 128
// OLED Displayhöhe in Pixel
#define SCREEN_HEIGHT 64
// Reset Pin # (-1 wenn ESP32 Reset Pin geteilt)
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         &Wire, OLED_RESET);
unsigned long lastOledUpdate = 0;
// Millisekunden
const long OLED_UPDATE_INTERVAL = 250;

// ----------------------------
// Bluepad32 Controller Verwaltung
// ----------------------------
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool controllerConnected = false;
unsigned long lastControllerActivity = 0;
// 5 Sekunden
const unsigned long CONTROLLER_TIMEOUT_MS = 5000;

// ----------------------------
// Roboter Zustandsmaschine (State Machine)
// ----------------------------
enum RobotState
{
    STANDBY,
    // Wartet auf Linienerkennung zum Starten
    LINE_FOLLOW_READY,
    // Folgt aktiv der Linie
    LINE_FOLLOWING
};
RobotState currentRobotState = STANDBY;
String getRobotStateName(RobotState state) {
    switch (state) {
        case STANDBY:           return "Standby";
        case LINE_FOLLOW_READY: return "LF Bereit"; // Angepasst für Anzeige
        case LINE_FOLLOWING:    return "LF Aktiv";  // Angepasst für Anzeige
        default:                return "Unbekannt";
    }
}

// ----------------------------
// Linienfolger Geschwindigkeiten (0-100%)
// ----------------------------
// Normale Vorwärtsgeschwindigkeit
const int NORM_GESCHW = 30;
// Geschw. schnelles Rad bei Korrektur
const int KORR_GESCHW_SCHNELL = 40;
// Geschw. langsames Rad bei Korrektur
const int KORR_GESCHW_LANGSAM = 20;
// Geschw. für Drehung bei nur einem Außensensor aktiv
// (optional)
const int TURN_GESCHW = 35;

// ----------------------------
// Sensor Zustände
// ----------------------------
// true wenn Linie erkannt
bool sL_val = false;
// true wenn Linie erkannt
bool sM_val = false;
// true wenn Linie erkannt
bool sR_val = false;

// Forward-Deklarationen (Funktionsprototypen)
void updateOLED();
void stopMotors();

// --------------------------------------------------------------------
// MOTORSTEUERUNG (PWM an INx-Pins, ENA/ENB auf HIGH gejumpert)
// Seite: 0 für links, 1 für rechts
// Geschw.: -100 (voll zurück) bis 100 (voll vor)
// --------------------------------------------------------------------
void setMotor(int side, int speedPct)
{
    // PWM-Kanäle für IN-Pins
    uint8_t ch_A, ch_B;

    // Linker Motor
    if (side == 0)
    {
        ch_A = CH_M_L_IN1;
        ch_B = CH_M_L_IN2;
    }
    // Rechter Motor
    else
    {
        ch_A = CH_M_R_IN3;
        ch_B = CH_M_R_IN4;
    }

    int dutyCycle = map(abs(speedPct), 0, 100, 0, PWM_MAX_MOT);
    dutyCycle = constrain(dutyCycle, 0, PWM_MAX_MOT);

    // Vorwärts
    if (speedPct > 0)
    {
        ledcWrite(ch_A, dutyCycle);
        ledcWrite(ch_B, 0);
    }
    // Rückwärts
    else if (speedPct < 0)
    {
        ledcWrite(ch_A, 0);
        ledcWrite(ch_B, dutyCycle);
    }
    // Stopp (Leerlauf)
    else
    {
        ledcWrite(ch_A, 0);
        ledcWrite(ch_B, 0);
    }
}

void stopMotors()
{
    // Linken Motor stoppen
    setMotor(0, 0);
    // Rechten Motor stoppen
    setMotor(1, 0);
    Serial.println("Motoren GESTOPPT"); // Debug-Ausgabe
}

// ----------------------------
// Bluepad32 Verbindungs-Callbacks
// ----------------------------
void onCtrlConnect(ControllerPtr ctl)
{
    bool slotFound = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == nullptr)
        {
            Serial.printf("Controller verbunden, idx=%d\n", i);
            myControllers[i] = ctl;
            slotFound = true;
            break;
        }
    }
    if (!slotFound) {
        Serial.println("Controller verbunden, aber kein freier Slot");
    }
    controllerConnected = true;
    lastControllerActivity = millis();
    updateOLED();
}

void onCtrlDisconnect(ControllerPtr ctl)
{
    bool ctrlFound = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == ctl)
        {
            Serial.printf("Controller getrennt, idx=%d\n", i);
            myControllers[i] = nullptr;
            ctrlFound = true;
            break;
        }
    }
    if (!ctrlFound) {
        Serial.println("Ctrl getrennt, nicht in Liste gefunden");
    }

    // Prüfen, ob noch ein Controller verbunden ist
    controllerConnected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            controllerConnected = true;
            break;
        }
    }

    if (!controllerConnected) {
        Serial.println("Alle Controller getrennt. Motoren stoppen");
        Serial.println("und Rückkehr in Standby.");
        stopMotors();
        currentRobotState = STANDBY;
    }
    updateOLED();
}

// ----------------------------
// Sensor Lese-Funktion
// ----------------------------
void readSensorStates_old()
{
    // KY-033: LOW = Linie (dunkel),
    // HIGH = Keine Linie / Boden (hell)
    // Wir speichern 'true', wenn Linie erkannt.
    sL_val = (digitalRead(SENSOR_L_PIN) == LOW);
    sM_val = (digitalRead(SENSOR_M_PIN) == LOW);
    sR_val = (digitalRead(SENSOR_R_PIN) == LOW);  
}

// ----------------------------
// Sensor Lese-Funktion
// ----------------------------
void readSensorStates()
{
    // Assuming KY-033 still outputs:
    // HIGH = Dunkler Untergrund
    // LOW = Heller Untergrund (z.B. weiße Linie)
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
    String ctrlStatus = controllerConnected ? "Verbunden" : "Getrennt";
    display.println(ctrlStatus);

    display.print("Modus: ");
    display.println(getRobotStateName(currentRobotState));

    display.print("Sens: L[");
    display.print(sL_val ? "1" : "0");
    display.print("] M[");
    display.print(sM_val ? "1" : "0");
    display.print("] R[");
    display.print(sR_val ? "1" : "0");
    display.println("]");

    bool lineVisible = sL_val || sM_val || sR_val;
    if (currentRobotState == LINE_FOLLOWING ||
        currentRobotState == LINE_FOLLOW_READY) {
        display.print("Linie: ");
        String lineStatus = lineVisible ? "Ja" : "Nein";
        display.println(lineStatus);
    }

    display.display();
}

// ----------------------------
// Controller Eingabe Verarbeitung
// ----------------------------
void processControllerInputs()
{
    ControllerPtr ctl = nullptr;
    // Ersten verbundenen Controller verwenden
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            ctl = myControllers[i];
            break;
        }
    }

    // Nur verarbeiten, wenn ein Controller gefunden wurde UND neue Daten vorhanden sind
    if (ctl && ctl->hasData()) {
        // Aktivitätszeitstempel aktualisieren
        lastControllerActivity = millis();

        // "Quadrat" (Square)-Taste (PS5) startet Linienfolgemodus.
        // Die Bluepad32 Bibliothek verwendet ctl->x() für die Quadrat-Taste.
        if (ctl->x()) { // Quadrat-Taste (Square) verwenden
            if (currentRobotState == STANDBY) {
                currentRobotState = LINE_FOLLOW_READY;
                Serial.println(
                    "Modus -> LF Bereit (Quadrat-Taste, wartet auf Linie)");
                // Sicherstellen, dass Motoren gestoppt sind
                stopMotors();
                // Einfaches Entprellen
                delay(200); // Nach dem Drücken kurz warten, um Mehrfachauslösung zu vermeiden
            }
        }

        // "Kreuz" (X)-Taste als Not-Aus
        // Compiler schlug BUTTON_A vor. Dies ist die generische Taste A,
        // die oft der PlayStation Kreuz-Taste (X) entspricht.
        // Bluepad32 verwendet ctl->a() für die Kreuz-Taste.
        if (ctl->a()) { // BUTTON_A (oder ctl->a() für explizite Kreuz-Taste) verwenden
            Serial.println("NOT-AUS gedrückt (via Kreuz-Taste)!");
            stopMotors();
            currentRobotState = STANDBY;
            Serial.println("Modus -> STANDBY");
            // Einfaches Entprellen
            delay(200);
        }
    }
}
// ----------------------------
// Linienfolge-Algorithmus
// ----------------------------
void executeLineFollowingAlgorithm()
{
    // Sensorwerte aktualisieren
    readSensorStates();

    // Linie verloren: Wenn alle Sensoren Boden sehen (keine Linie)
    if (!sL_val && !sM_val && !sR_val) {
        // Stoppen, aber im Linienfolgemodus bleiben
        stopMotors();
        Serial.println(
            "Linie verloren, Motoren gestoppt. Warte auf Linie...");
        return;
    }

    // Linienfolge-Logik:
    // Mittlerer Sensor auf Linie: Geradeaus fahren
    if (sM_val) {
        Serial.println("Linie: Mitte -> Geradeaus");
        // Linker Motor
        setMotor(0, NORM_GESCHW);
        // Rechter Motor
        setMotor(1, NORM_GESCHW);
    }
    // Nur linker Sensor auf Linie (oder Links und Mitte,
    // aber Mitte oben behandelt): Nach links korrigieren
    else if (sL_val && !sR_val) {
        Serial.println("Linie: Links -> Nach Links");
        // Linker Motor langsamer
        setMotor(0, KORR_GESCHW_LANGSAM);
        // Rechter Motor schneller
        setMotor(1, KORR_GESCHW_SCHNELL);
    }
    // Nur rechter Sensor auf Linie (oder Rechts und Mitte,
    // aber Mitte oben behandelt): Nach rechts korrigieren
    else if (sR_val && !sL_val) {
        Serial.println("Linie: Rechts -> Nach Rechts");
        // Linker Motor schneller
        setMotor(0, KORR_GESCHW_SCHNELL);
        // Rechter Motor langsamer
        setMotor(1, KORR_GESCHW_LANGSAM);
    }
    else {
        // Fallback / Undefinierter Zustand (z.B. L und R sehen Linie,
        // aber M nicht).
        // Dies kann passieren, wenn die Linie sehr breit ist oder
        // an einer 'T'-Kreuzung oder Abzweigung.
        // Vorerst einfacher Stopp.
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
    Serial.println();
    Serial.println("Initialisiere " + ROBOT_NAME + "...");

    // I2C für OLED initialisieren
    // Verwendet Standard ESP32 I2C Pins (SDA=21, SCL=22)
    Wire.begin();
    Serial.println("I2C Bus initialisiert.");

    // OLED Display initialisieren
    Serial.println("Initialisiere SSD1306 OLED Display...");
    bool displayOk = display.begin(SSD1306_SWITCHCAPVCC,
                                   SCREEN_ADDRESS);
    if (!displayOk)
    {
        Serial.print(F("SSD1306 Allokierung fehlgeschlagen oder "));
        Serial.println(F("Display nicht gefunden!"));
        // Anhalten
        while (1) delay(10);
    }
    // Um 180 Grad für Überkopfmontage drehen
    display.setRotation(2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(ROBOT_NAME);
    display.println("Initialisiere...");
    display.display();
    Serial.println("OLED Display initialisiert.");

    // Motor INx Pins für PWM initialisieren
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
    // Sicherstellen, dass Motoren beim Start aus sind
    stopMotors();

    // Sensor-Pins initialisieren
    Serial.println("Initialisiere Liniensensoren...");
    pinMode(SENSOR_L_PIN, INPUT);
    pinMode(SENSOR_M_PIN, INPUT);
    pinMode(SENSOR_R_PIN, INPUT);
    Serial.println("Sensor-Pins als INPUT konfiguriert.");

    // Bluepad32 initialisieren
    Serial.println("Initialisiere Bluepad32...");
    BP32.setup(&onCtrlConnect, &onCtrlDisconnect);
    // Normalerweise false für physische Controller
    BP32.enableVirtualDevice(false);
    // Einkommentieren, um Pairing zu erzwingen
    // BP32.forgetBluetoothKeys();

    currentRobotState = STANDBY;
    Serial.println("Setup abgeschlossen. Roboter im STANDBY Modus.");
    // Initialen Status anzeigen
    updateOLED();
}

// ----------------------------
// HAUPTSCHLEIFE (LOOP)
// ----------------------------
void loop()
{
    // BP32.update() verarbeitet Bluetooth-Ereignisse.
    // Es löst onCtrlConnect/onCtrlDisconnect Callbacks aus
    // und ruft Gamepad-Daten für processControllerInputs() ab.
    if (BP32.update()) {
        // Arbeit erfolgt in Callbacks o. durch Prüfen d. Controller-Status.
    }

    if (controllerConnected) {
        processControllerInputs();

        // Controller-Timeout-Prüfung
        unsigned long timeDiff = millis() - lastControllerActivity;
        if (timeDiff > CONTROLLER_TIMEOUT_MS) {
            Serial.print("Controller Timeout. Motoren stoppen");
            Serial.println(" und Rückkehr in Standby.");
            stopMotors();
            currentRobotState = STANDBY;
            // Sofortiges Neutriggern verhindern
            lastControllerActivity = millis();
        }
    }

    // Aktionen der Roboter-Zustandsmaschine
    switch (currentRobotState)
    {
    case STANDBY:
        // Motoren gestoppt. Sensoren für Anzeige lesen.
        readSensorStates();
        break;

    case LINE_FOLLOW_READY:
        // Motoren gestoppt. Sensoren kontinuierlich prüfen.
        readSensorStates();
        // Wenn mittlerer Sensor Linie erkennt
        if (sM_val) {
            currentRobotState = LINE_FOLLOWING;
            Serial.print("Modus -> LF Aktiv ");
            Serial.println("(Mittlerer Sensor hat Linie erkannt)");
        }
        break;

    case LINE_FOLLOWING:
        executeLineFollowingAlgorithm();
        break;
    }

    // OLED-Display periodisch aktualisieren
    unsigned long currentTime = millis();
    if (currentTime - lastOledUpdate > OLED_UPDATE_INTERVAL)
    {
        // Sicherstellen, dass Sensorwerte für Anzeige aktuell sind
        if (currentRobotState != LINE_FOLLOWING) {
             readSensorStates();
        }
        updateOLED();
        lastOledUpdate = currentTime;
    }
    // Kurze Verzögerung für Stabilität
    delay(10);
}
