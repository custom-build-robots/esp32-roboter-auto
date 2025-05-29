/***************************************************
 * Autor: Ingmar Stapel
 * Webseite: www.custom-build-robots.com
 * Datu,: 2025-05-24
 * Version: 1.0 
 * ESP32 Servo-Steuerung mit PCA9685 und Adafruit-Bibliothek
 *
 * Dieses Programm steuert einen einzelnen analogen Servo-Motor,
 * der an Kanal 0 eines I2C-basierten PCA9685 Servo-Kontrollers
 * angeschlossen ist. Der ESP32 kommuniziert mit dem PCA9685
 * über I2C unter Verwendung der Adafruit PWMServoDriver Bibliothek.
 *
 * Funktionen:
 * - Initialisierung des PCA9685 Servo-Kontrollers.
 * - Ansteuerung eines Servo-Motors an Kanal 0.
 * - Vordefinierte Bewegungssequenz:
 * 1. Drehung auf Mittelstellung.
 * 2. Drehung um 45° nach links von der Mitte.
 * 3. Drehung zurück auf Mittelstellung.
 * 4. Drehung um 45° nach rechts von der Mitte.
 * - Die Sequenz wird 10-mal wiederholt.
 * - Nach Abschluss der Sequenz stoppt der Servo in der Mittelstellung.
 *
 * Benötigte Bibliotheken:
 * - Wire.h (Standard für ESP32 I2C)
 * - Adafruit_PWMServoDriver.h
 *
 ********************************************************************/
 
 #include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Erzeuge ein Objekt für den PCA9685 Servo Controller
// Standard I2C Adresse ist 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Wenn du eine andere Adresse verwendest, z.B. 0x41:
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Definiere die Pulsbreiten für den Servo (in Mikrosekunden)
// Diese Werte müssen eventuell für deinen spezifischen Servo angepasst werden!
#define SERVOMIN_US 600  // Minimale Pulsbreite (entspricht oft 0 Grad)
#define SERVOMAX_US 2400 // Maximale Pulsbreite (entspricht oft 180 Grad)
#define SERVO_FREQ 50    // Frequenz für analoge Servos (typischerweise 50 Hz)

// Servokanal, an dem der Motor angeschlossen ist (0-15)
#define SERVO_CHANNEL 0

// Berechnung der Pulsbreiten für die Zielpositionen
// Mittelstellung (angenommen als 90 Grad)
const int PULSE_CENTER = (SERVOMIN_US + SERVOMAX_US) / 2;
// Versatz für 45 Grad Drehung
// (SERVOMAX_US - SERVOMIN_US) ist der Pulsbereich für 180 Grad
// Also ist der Puls für 45 Grad: (Pulsbereich / 180) * 45  oder einfacher (Pulsbereich / 4)
const int PULSE_45_DEG_OFFSET = (SERVOMAX_US - SERVOMIN_US) / 4;

const int POS_MITTE = PULSE_CENTER;
const int POS_LINKS_45 = PULSE_CENTER - PULSE_45_DEG_OFFSET; // Von Mitte -45°
const int POS_RECHTS_45 = PULSE_CENTER + PULSE_45_DEG_OFFSET; // Von Mitte +45°

// Anzahl der Wiederholungen der Sequenz
#define LOOP_COUNT 10

// Verzögerung zwischen den Servobewegungen in Millisekunden
// Gib dem Servo Zeit, die Position zu erreichen
#define MOVE_DELAY 700

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 PCA9685 Servo Test");

  // Initialisiere den PCA9685
  pwm.begin();
  // Optional: Setze die Oszillatorfrequenz, falls bekannt und für Präzision nötig
  // pwm.setOscillatorFrequency(27000000); // Beispielwert, siehe Datenblatt PCA9685
  pwm.setPWMFreq(SERVO_FREQ); // Setze die PWM-Frequenz für Servos

  delay(100); // Kurze Pause

  Serial.println("Starte Servo-Bewegungssequenz...");

  for (int i = 0; i < LOOP_COUNT; i++) {
    Serial.print("Durchlauf: ");
    Serial.println(i + 1);

    // 1. Auf Mittelstellung drehen
    Serial.println("- Mittelstellung");
    pwm.writeMicroseconds(SERVO_CHANNEL, POS_MITTE);
    delay(MOVE_DELAY);

    // 2. Um 45° nach links drehen
    Serial.println("- 45° nach links");
    pwm.writeMicroseconds(SERVO_CHANNEL, POS_LINKS_45);
    delay(MOVE_DELAY);

    // 3. Zurück auf die Mittelstellung
    Serial.println("- Zurück zur Mittelstellung");
    pwm.writeMicroseconds(SERVO_CHANNEL, POS_MITTE);
    delay(MOVE_DELAY);

    // 4. Um 45° nach rechts drehen
    Serial.println("- 45° nach rechts");
    pwm.writeMicroseconds(SERVO_CHANNEL, POS_RECHTS_45);
    delay(MOVE_DELAY);

    // Am Ende des Durchlaufs (außer dem letzten) zurück zur Mitte für sauberen Start
    if (i < LOOP_COUNT - 1) {
      pwm.writeMicroseconds(SERVO_CHANNEL, POS_MITTE);
      delay(MOVE_DELAY);
    }
  }

  // Nach 10 Durchläufen Servo auf Mittelstellung und dann stoppen
  Serial.println("Sequenz beendet. Servo auf Mittelstellung und gestoppt.");
  pwm.writeMicroseconds(SERVO_CHANNEL, POS_MITTE);
  // Um den Servo wirklich "auszuschalten" (er hält dann nicht mehr aktiv die Position):
  // pwm.setPWM(SERVO_CHANNEL, 0, 0); // Sendet kein PWM Signal mehr an diesen Kanal
}

void loop() {
  // Die Hauptlogik ist in setup() ausgeführt worden und wird nur einmal durchlaufen.
  // loop() bleibt leer oder kann für andere Aufgaben genutzt werden.
  // Der Servo bleibt in der letzten Position (Mittelstellung).
  delay(1000); // Verhindert, dass loop() zu schnell rotiert, falls doch was rein kommt.
}