/***************************************************
 * Autor: Ingmar Stapel
 * Webseite: www.custom-build-robots.com
 * Datu,: 2025-05-27
 * Version: 1.0 
 * Beschreibung:
 *
 * Dieses ESP32-Programm misst die Umgebungslichtstärke mit einem
 * BH1750 I2C-Lichtsensor und zeigt den Wert in Lux auf einem
 * SSD1306 I2C OLED-Display an.
 * Es verwendet die BH1750-Bibliothek (von Christopher Laws) und
 * Adafruits GFX- und SSD1306-Bibliotheken für die Displaysteuerung.
 * Das Programm initialisiert beide I2C-Geräte, liest den Licht-
 * sensor etwa jede Sekunde aus und zeigt das Ergebnis auf
 * dem OLED an. Serielle Ausgabe zum Debuggen ist vorhanden, und
 * eine grundlegende Fehlerbehandlung für Sensor-/Display-Initialisierung
 * und Sensormessungen ist implementiert.
 *
 * Hardware:
 * - ESP32 Microcontroller
 * - BH1750 Light Sensor (I2C)
 * - SSD1306 OLED Display (I2C)
 *
 * I2C Pins (ESP32 default):
 * - SDA: GPIO 21
 * - SCL: GPIO 22
 *
 ****************************************************/

// Benötigte Bibliotheken einbinden
#include <Wire.h>                 // Für I2C-Kommunikation
#include <BH1750.h>               // Für den BH1750 Lichtsensor (von Christopher Laws)
#include <Adafruit_GFX.h>         // Basis-Grafikbibliothek für das Display
#include <Adafruit_SSD1306.h>     // Für das SSD1306 OLED Display

// I2C-Pinbelegung (Standard für die meisten ESP32-Boards)
// Diese können bei Bedarf angepasst werden.
// const int I2C_SDA_PIN = 21; // GPIO21
// const int I2C_SCL_PIN = 22; // GPIO22

// OLED Display Konfiguration
#define SCREEN_WIDTH 128 // OLED Display Breite in Pixeln
#define SCREEN_HEIGHT 64 // OLED Display Höhe in Pixeln
#define OLED_RESET    -1 // Reset Pin # (oder -1 wenn der Reset-Pin des Displays mit dem ESP32 Reset verbunden ist)
// Die I2C-Adresse des SSD1306 ist oft 0x3C oder 0x3D.
// Wenn das Display nicht erkannt wird, versuche die andere Adresse.
#define SCREEN_ADDRESS 0x3C

// Objekte für Sensor und Display erstellen
BH1750 lightMeter; // BH1750 Objekt
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // SSD1306 Objekt

// Globale Variable für den letzten Zeitpunkt der Display-Aktualisierung
unsigned long previousMillis = 0;
const long interval = 1000; // Intervall für die Display-Aktualisierung (in Millisekunden)

void setup() {
  // Serielle Kommunikation für Debugging starten
  Serial.begin(115200);
  Serial.println("BH1750 & SSD1306 Testprogramm");

  // I2C-Bus initialisieren
  // Wire.begin(); // Standardinitialisierung, verwendet die Standard-Pins
  // Alternativ, falls benutzerdefinierte Pins benötigt werden:
  // Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Nutzt die Standard I2C Pins des ESP32 (GPIO 21 SDA, GPIO 22 SCL)
  Wire.begin(); 

  // BH1750 Sensor initialisieren
  Serial.println("Initialisiere BH1750 Sensor...");
  // Der BH1750 kann zwei Adressen haben: 0x23 (Standard) oder 0x5C (wenn ADDR-Pin auf HIGH)
  // Die Bibliothek versucht standardmäßig 0x23.
  // Wenn der Sensor eine andere Adresse hat: lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C);
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 Sensor erfolgreich initialisiert.");
  } else {
    Serial.println("Fehler: BH1750 Sensor nicht gefunden oder Initialisierung fehlgeschlagen!");
    // Hier könnte man eine Dauerschleife einbauen oder eine Fehlermeldung auf dem Display anzeigen,
    // falls das Display bereits initialisiert wäre. Da es noch nicht initialisiert ist,
    // beschränken wir uns auf die serielle Ausgabe.
    while (1) delay(10); // Anhalten bei Fehler
  }

  // SSD1306 Display initialisieren
  Serial.println("Initialisiere SSD1306 OLED Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Fehler: SSD1306 Display nicht gefunden!"));
    // Programm anhalten, wenn das Display nicht initialisiert werden kann.
    while (1) delay(10);
  }
  Serial.println("SSD1306 OLED Display erfolgreich initialisiert.");

  // Display-Inhalt löschen
  display.clearDisplay();

  // Textfarbe setzen (weiß auf schwarzem Hintergrund)
  display.setTextColor(SSD1306_WHITE);
  // Textgröße setzen
  display.setTextSize(1);
  // Cursorposition setzen
  display.setCursor(0,0);
  display.println("BH1750 & SSD1306");
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

    float lux = lightMeter.readLightLevel(); // Lichtwert auslesen

    // Überprüfen, ob der Messwert gültig ist
    // Der Sensor gibt negative Werte zurück, 
    // wenn ein Timeout während der Messung auftritt
    // oder wenn der Messwert außerhalb des gültigen Bereichs liegt.
    if (lux < 0) {
      Serial.println("Fehler beim Auslesen des BH1750 Sensors!");
      // Fehlermeldung auf dem Display anzeigen
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("Sensor Fehler!");
      display.display();
    } else {
      // Lichtwert auf dem seriellen Monitor ausgeben
      Serial.print("Licht: ");
      Serial.print(lux);
      Serial.println(" Lux");

      // Display-Inhalt löschen
      display.clearDisplay();

      // Textgröße und Cursorposition für die Ausgabe setzen
      display.setTextSize(1); // Größerer Text für den Messwert
      display.setCursor(0, 10); // Etwas nach unten verschoben

      // Text für das Display formatieren
      // Umwandlung in Integer für eine saubere Anzeige ohne Nachkommastellen
      String displayText = "Licht: " + String((int)lux) + " Lux"; 

      // Text auf dem Display anzeigen
      display.println(displayText);

      // Inhalt auf dem Display tatsächlich darstellen
      display.display();
    }

    // Erneute Messung für den kontinuierlichen Modus anfordern (optional,
    // da die Bibliothek dies im kontinuierlichen Modus möglicherweise 
    // intern handhabt, aber es schadet nicht und stellt sicher, 
    // dass die nächste Messung gestartet wird)
    // Nicht unbedingt nötig, wenn einmal gesetzt
    // lightMeter.configure(BH1750::CONTINUOUS_HIGH_RES_MODE); 
    
  }
}