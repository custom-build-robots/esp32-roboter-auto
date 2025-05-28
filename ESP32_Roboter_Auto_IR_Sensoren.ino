/*******************************************************************************
 * Autor: Roboter-Enthusiast (Bitte anpassen)
 * Webseite: www.custom-build-robots.com (Bitte anpassen)
 * Datum: 28. Mai 2025
 * Version: 1.0
 * Test: OK (Grundfunktion der Sensoranzeige)
 *
 * Beschreibung:
 * Dieses ESP32-Programm dient als Test und Demonstration für drei KY-033
 * Linienfolgesensoren in Verbindung mit einem SSD1306 I2C OLED-Display.
 * Es liest periodisch (jede Sekunde) die digitalen Zustände der drei an
 * den ESP32 angeschlossenen KY-033 Sensoren aus. Die erfassten Zustände
 * (typischerweise LOW für "Linie erkannt" und HIGH für "Boden/keine Linie")
 * werden sowohl auf dem seriellen Monitor für Debugging-Zwecke als auch
 * formatiert auf dem OLED-Display ausgegeben. Das OLED-Display ist für
 * eine um 180 Grad gedrehte Montage konfiguriert.
 * Dieses Programm beinhaltet keine Motorsteuerung oder Linienfolge-Logik,
 * sondern fokussiert sich rein auf die Sensordatenerfassung und -anzeige.
 *
 * Hardware-Setup:
 * - ESP32 Mikrocontroller
 * - 3x KY-033 Linienfolgesensoren:
 * - Linker Sensor: GPIO 34
 * - Mittlerer Sensor: GPIO 2
 * - Rechter Sensor: GPIO 35
 * - SSD1306 OLED Display (128x64, I2C, Adresse 0x3C)
 * - I2C SDA: Standard ESP32 I2C Pin (z.B. GPIO 21)
 * - I2C SCL: Standard ESP32 I2C Pin (z.B. GPIO 22)
 *
 * Kernfunktionen:
 * - Initialisierung der GPIO-Pins für die KY-033 Sensoren als Eingänge.
 * - Initialisierung des I2C-Busses und des SSD1306 OLED-Displays,
 * inklusive Drehung der Anzeige um 180 Grad.
 * - Periodisches Auslesen der digitalen Zustände (HIGH/LOW) der drei
 * Linienfolgesensoren im Hauptloop (Intervall: 1 Sekunde).
 * - Umwandlung der Sensorzustände in lesbare Texte ("Linie" / "Boden").
 * - Ausgabe der Sensorzustände (Text und Rohwert) auf dem seriellen Monitor.
 * - Anzeige der Sensorzustände (Text) auf dem OLED-Display.
 *
 *******************************************************************************/


// Benötigte Bibliotheken einbinden
#include <Wire.h>             // Für I2C-Kommunikation (OLED Display)
#include <Adafruit_GFX.h>     // Basis-Grafikbibliothek für das Display
#include <Adafruit_SSD1306.h> // Für das SSD1306 OLED Display

// Pin-Definitionen für die KY-033 Linienfolgesensoren
const int SENSOR_L_PIN = 34; // Linker Sensor (Sensor 1)
const int SENSOR_M_PIN = 2;  // Mittlerer Sensor (Sensor 2)
const int SENSOR_R_PIN = 35;  // Rechter Sensor (Sensor 3)

// OLED Display Konfiguration
#define SCREEN_WIDTH 128 // OLED Display Breite in Pixeln
#define SCREEN_HEIGHT 64 // OLED Display Höhe in Pixeln
#define OLED_RESET    -1 // Reset Pin # (oder -1 wenn der Reset-Pin des Displays mit dem ESP32 Reset verbunden ist)
// Die I2C-Adresse des SSD1306 ist oft 0x3C oder 0x3D.
// Wenn das Display nicht erkannt wird, versuche die andere Adresse.
#define SCREEN_ADDRESS 0x3C

// Objekt für das Display erstellen
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Globale Variable für den letzten Zeitpunkt der Display-Aktualisierung
unsigned long previousMillis = 0;
const long interval = 1000; // Intervall für die Display-Aktualisierung (in Millisekunden)

void setup() {
  // Serielle Kommunikation für Debugging starten
  Serial.begin(115200);
  Serial.println("KY-033 Linienfolger & SSD1306 Testprogramm");

  // GPIO-Pins für die Sensoren als Eingänge konfigurieren
  // KY-033 Module liefern oft bereits ein sauberes HIGH/LOW Signal,
  // daher ist INPUT meist ausreichend. Falls die Signale "flattern" oder
  // nicht eindeutig sind, könnte man INPUT_PULLUP testen, was aber
  // die Logik umkehren würde, wenn der Sensor LOW bei Linie ausgibt.
  // Für die typische Anwendung (LOW = Linie) ist INPUT korrekt.
  pinMode(SENSOR_L_PIN, INPUT);
  pinMode(SENSOR_M_PIN, INPUT);
  pinMode(SENSOR_R_PIN, INPUT);
  Serial.println("Sensor-Pins als INPUT konfiguriert.");

  // I2C-Bus initialisieren (für das OLED-Display)
  // Wire.begin(); // Standardinitialisierung, verwendet die Standard-Pins (GPIO 21 SDA, GPIO 22 SCL für ESP32)
  // Alternativ, falls benutzerdefinierte Pins benötigt werden (selten für Standard ESP32 Dev Kits):
  // Wire.begin(SDA_PIN, SCL_PIN);
  Wire.begin(); 
  Serial.println("I2C-Bus initialisiert.");

  // SSD1306 Display initialisieren
  Serial.println("Initialisiere SSD1306 OLED Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Fehler: SSD1306 Allokierung fehlgeschlagen oder Display nicht gefunden!"));
    // Programm anhalten, wenn das Display nicht initialisiert werden kann.
    while (1) delay(10);
  }
  Serial.println("SSD1306 OLED Display erfolgreich initialisiert.");

  // WICHTIG: Display-Ausrichtung um 180 Grad drehen (für Über-Kopf-Montage)
  // Rotation: 0 = 0 Grad, 1 = 90 Grad, 2 = 180 Grad, 3 = 270 Grad
  display.setRotation(2); 
  Serial.println("Display-Rotation auf 180 Grad gesetzt.");

  // Display-Inhalt löschen
  display.clearDisplay();

  // Textfarbe setzen (weiß auf schwarzem Hintergrund)
  display.setTextColor(SSD1306_WHITE);
  // Textgröße setzen
  display.setTextSize(1);
  // Cursorposition setzen
  display.setCursor(0,0);
  display.println("KY-033 Test");
  display.println("Initialisierung...");
  display.display(); // Inhalt auf dem Display anzeigen
  delay(2000); // Kurze Pause, um die Initialisierungsnachricht zu sehen
}

void loop() {
  // Aktuelle Zeit erfassen
  unsigned long currentMillis = millis();

  // Nur etwa jede Sekunde aktualisieren
  if (currentMillis - previousMillis >= interval) {
    // Zeitstempel für die letzte Aktualisierung speichern
    previousMillis = currentMillis;

    // Digitale Zustände der Sensoren auslesen
    // KY-033: LOW = Linie (dunkel), HIGH = keine Linie / heller Untergrund
    bool stateL = digitalRead(SENSOR_L_PIN);
    bool stateM = digitalRead(SENSOR_M_PIN);
    bool stateR = digitalRead(SENSOR_R_PIN);

    // Zustände in Text umwandeln
    String statusL = (stateL == LOW) ? "Linie" : "Boden";
    String statusM = (stateM == LOW) ? "Linie" : "Boden";
    String statusR = (stateR == LOW) ? "Linie" : "Boden";
    // Alternativ zu "Boden" könnte "Hell" oder "Keine Linie" verwendet werden.
    // String statusL = (stateL == LOW) ? "Linie" : "Keine Linie";
    // String statusM = (stateM == LOW) ? "Linie" : "Keine Linie";
    // String statusR = (stateR == LOW) ? "Linie" : "Keine Linie";

    // Sensorzustände auf dem seriellen Monitor ausgeben
    Serial.print("Sensor 1 (Links): "); Serial.print(statusL); Serial.print(" (Pin Val: "); Serial.print(stateL); Serial.println(")");
    Serial.print("Sensor 2 (Mitte): "); Serial.print(statusM); Serial.print(" (Pin Val: "); Serial.print(stateM); Serial.println(")");
    Serial.print("Sensor 3 (Rechts): "); Serial.print(statusR); Serial.print(" (Pin Val: "); Serial.print(stateR); Serial.println(")");
    Serial.println("---");

    // Display-Inhalt löschen
    display.clearDisplay();

    // Textgröße und Cursorposition für die Ausgabe setzen
    display.setTextSize(1); // Normale Textgröße für mehrere Zeilen
    
    display.setCursor(0, 0); // Start erste Zeile
    display.print("Sensor 1: ");
    display.println(statusL);

    display.setCursor(0, 10); // Start zweite Zeile (Pixelabstand anpassen bei Bedarf)
    display.print("Sensor 2: ");
    display.println(statusM);

    display.setCursor(0, 20); // Start dritte Zeile
    display.print("Sensor 3: ");
    display.println(statusR);

    // Inhalt auf dem Display tatsächlich darstellen
    display.display();
  }
}
