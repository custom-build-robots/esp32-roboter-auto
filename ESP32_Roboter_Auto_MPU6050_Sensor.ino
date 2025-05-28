/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2025-05-27
 * Version: 1.9
 * Description:
 *
 * Dieses ESP32-Programm liest Daten von einem MPU6050 Sensor aus,
 * der Beschleunigung, Drehrate (Gyroskop) und Temperatur misst.
 * Die Messwerte für alle drei Achsen der Beschleunigung (in m/s²),
 * des Gyroskops (in rad/s) sowie die Temperatur (in °C)
 * werden auf einem SSD1306 I2C OLED-Display dargestellt.
 * Das Programm verwendet die Adafruit MPU6050, Adafruit Sensor,
 * Adafruit GFX und Adafruit SSD1306 Bibliotheken. Die Anzeige
 * der Daten wird etwa alle 100 Millisekunden aktualisiert.
 * Serielle Ausgaben unterstützen das Debugging und zeigen den
 * Initialisierungsstatus der Komponenten an.
 *
 * Hardware-Setup:
 * - ESP32 Mikrocontroller
 * - MPU6050 Sensor Modul (I2C)
 * - SSD1306 OLED Display (I2C, typ. Adresse 0x3C)
 * - I2C SDA: Standard ESP32 I2C Pin (z.B. GPIO 21)
 * - I2C SCL: Standard ESP32 I2C Pin (z.B. GPIO 22)
 *
 ****************************************************/
/// Bibliotheken, die verwendet werden:
// - Wire.h: Für die I2C-Kommunikation.
// - Adafruit_MPU6050.h: Bibliothek für den MPU6050 Sensor.
// - Adafruit_Sensor.h: Basis-Sensorbibliothek von Adafruit (Abhängigkeit für Adafruit_MPU6050).
// - Adafruit_GFX.h: Basis-Grafikbibliothek für das OLED-Display.
// - Adafruit_SSD1306.h: Bibliothek für das SSD1306 OLED-Display.

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// MPU6050 Sensor Objekt
Adafruit_MPU6050 mpu;

// OLED Display Einstellungen
#define SCREEN_WIDTH 128 // OLED Display Breite in Pixeln
#define SCREEN_HEIGHT 64 // OLED Display Höhe in Pixeln
#define OLED_RESET -1    // Reset Pin (oder -1, wenn nicht verwendet/geteilt)
// Standard I2C Adresse für viele SSD1306 Displays ist 0x3C
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // Serielle Kommunikation für Debugging starten
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Warte bis die serielle Verbindung steht (für einige Boards nötig)
  }
  Serial.println("MPU6050 & SSD1306 Testprogramm");

  // I2C-Bus initialisieren (Standardpins des ESP32 für SDA, SCL)
  Wire.begin();

  // OLED-Display initialisieren
  Serial.println("Initialisiere OLED Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("Fehler: SSD1306 Allokierung fehlgeschlagen oder Display nicht gefunden!"));
    // Programm anhalten, wenn das Display nicht initialisiert werden kann.
    // Eine Nachricht auf dem OLED ist hier nicht möglich.
    while (1) delay(10);
  }
  Serial.println("OLED Display erfolgreich initialisiert.");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("OLED OK");
  display.display();
  delay(500); // Kurze Anzeige der Initialisierungsnachricht

  // MPU-6050 initialisieren
  Serial.println("Initialisiere MPU6050 Sensor...");
  if (!mpu.begin()) {
    Serial.println("Fehler: MPU6050 nicht gefunden! Verkabelung prüfen.");
    // Fehlermeldung auf OLED ausgeben, falls Display funktioniert
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("MPU6050 Fehler:");
    display.println("Nicht gefunden!");
    display.display();
    // Programm anhalten
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 erfolgreich initialisiert!");
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("MPU6050 OK");
  display.display();
  delay(500);

  // Fortgeschrittene Einstellungen für MPU6050 sind optional und hier nicht implementiert.
  // Beispiele (bei Bedarf einkommentieren und anpassen):
  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Setup abgeschlossen. Starte Messungen...");
  display.clearDisplay(); // Display für Messwerte vorbereiten
  display.display();
}

void loop() {
  // Sensor-Events für Beschleunigung, Gyroskop und Temperatur erstellen
  sensors_event_t a, g, temp;
  // Neue Sensor-Events vom MPU6050 auslesen
  mpu.getEvent(&a, &g, &temp);

  // Display leeren und Text-Setup für neue Ausgabe
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); // Startposition oben links

  // Beschleunigungswerte anzeigen
  display.println("Beschl. (m/s^2):");
  display.print("X:");
  display.print(a.acceleration.x, 1); // 1 Nachkommastelle
  display.print(" Y:");
  display.print(a.acceleration.y, 1);
  display.print(" Z:");
  display.println(a.acceleration.z, 1); // println für Zeilenumbruch nach Z

  // Gyroskopwerte anzeigen
  // Neue Zeile durch println oben, oder Cursor manuell setzen
  // display.setCursor(0,18); // Ungefähre Y-Position für nächste Sektion
  display.println("Gyro (rad/s):");
  display.print("X:");
  display.print(g.gyro.x, 2); // 2 Nachkommastellen
  display.print(" Y:");
  display.print(g.gyro.y, 2);
  display.print(" Z:");
  display.println(g.gyro.z, 2); // println für Zeilenumbruch nach Z

  // Temperaturwert anzeigen
  // display.setCursor(0,36); // Ungefähre Y-Position für nächste Sektion
  display.print("Temp: ");
  display.print(temp.temperature, 1); // 1 Nachkommastelle
  display.println(" C");

  // Alle vorbereiteten Daten auf dem OLED anzeigen
  display.display();

  // Optionale serielle Ausgabe für detaillierteres Debugging
  // Serial.println("--- Sensorwerte ---");
  // Serial.print("Beschleunigung X: "); Serial.print(a.acceleration.x, 1);
  // Serial.print(" m/s^2, Y: "); Serial.print(a.acceleration.y, 1);
  // Serial.print(" m/s^2, Z: "); Serial.print(a.acceleration.z, 1); Serial.println(" m/s^2");
  //
  // Serial.print("Gyroskop X: "); Serial.print(g.gyro.x, 2);
  // Serial.print(" rad/s, Y: "); Serial.print(g.gyro.y, 2);
  // Serial.print(" rad/s, Z: "); Serial.print(g.gyro.z, 2); Serial.println(" rad/s");
  //
  // // Umrechnung Gyroskop von rad/s in Grad/s für serielle Ausgabe: Wert * (180/M_PI)
  // Serial.print("Gyroskop (Grad/s) X: "); Serial.print(g.gyro.x * (180/M_PI), 2);
  // Serial.print(", Y: "); Serial.print(g.gyro.y * (180/M_PI), 2);
  // Serial.print(", Z: "); Serial.println(g.gyro.z * (180/M_PI), 2);
  //
  // Serial.print("Temperatur: "); Serial.print(temp.temperature, 1); Serial.println(" C");
  // Serial.println("---------------------");

  // Kurze Pause zwischen den Messungen (ca. alle 100 Millisekunden)
  delay(100);
}
