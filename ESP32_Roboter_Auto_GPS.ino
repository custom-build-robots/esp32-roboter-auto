// Autor: Ingmar Stapel
// Homepage: https://custom-build-robots.com/
// Date: 2020-10-30
//
// Rewritten for ESP32 with improvements by Gemini
// - Replaced SoftwareSerial with HardwareSerial for better stability on ESP32.
// - Restructured the main loop for more efficient display updates.
// - Added a check for missing GPS signal to aid in debugging.
// - Centralized pin configuration.

#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h> // Korrigiert: Die Adafruit Bibliothek wird nun verwendet.
#include <Adafruit_GFX.h>     // Wird für die Zeichenfunktionen benötigt.

// --- PIN Configuration ---
// Default I2C pins for the OLED display on most ESP32 boards are:
#define OLED_SDA 21
#define OLED_SCL 22

// Define the RX and TX pins for the GPS module connected to Serial2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// --- Adafruit Display Definition (NEU) ---
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET   -1  // Reset pin # (oder -1, wenn der Arduino Reset-Pin geteilt wird)
#define SCREEN_ADDRESS 0x3C ///< I2C-Adresse

// --- GPS and Display Objects ---
TinyGPSPlus gps;
// Use HardwareSerial for the GPS module. Serial2 is a common choice (UART2).
HardwareSerial gpsSerial(2);
// Initialisierung mit der Adafruit-Klasse (ersetzt SSD1306Wire)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Global Variables ---
// Set your local time zone offset from UTC
const int TIME_ZONE_HOUR_OFFSET = 2; // e.g., 2 for CEST
const int TIME_ZONE_MINUTE_OFFSET = 0;

// --- Function Prototypes ---
void updateDisplayData();
void drawOLED(const String& datetime_str, const String& lat_str, const String& lng_str);

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(115200);
  Serial.println("GPS OLED Display starting up...");

  // Start the hardware serial port for the GPS module
  // The begin method takes (baud_rate, protocol, RX_PIN, TX_PIN)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Serial connection started on pins RX:16, TX:17");

  // --- OLED Initialisierung (angepasst für Adafruit) ---
  // Die Wire-Library (I2C) wird automatisch gestartet.
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed
  }
  
  // Standard-Einstellungen für Adafruit GFX (kleinste Schrift, weißer Text)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // display.flipScreenVertically(); ist nicht Adafruit-Standard.
  // display.setRotation(2); würde 180° Drehung bewirken.
  
  display.display(); // Zeigt den Adafruit-Splashscreen
  Serial.println("OLED Display Initialized.");
}

void loop() {
  // Continuously read from the GPS serial port and feed the data to the TinyGPS++ object
  while (gpsSerial.available() > 0) {
    // When gps.encode() returns true, a new valid sentence has been received
    if (gps.encode(gpsSerial.read())) {
      updateDisplayData(); // Process and display the new data
    }
  }

  // If 5 seconds have passed and we still haven't received any GPS data,
  // display a warning. This is useful for debugging wiring issues.
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    // Korrektur der Zeichenbefehle für Adafruit GFX
    display.clearDisplay();
    
    // Schriftgröße 2 für "No GPS data" (größer)
    display.setTextSize(2); 
    display.setCursor(0, 0); 
    display.print("No GPS data");
    
    // Schriftgröße 1 für Details (kleiner)
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print("Check wiring to");
    display.setCursor(0, 36);
    display.print("GPS module.");
    
    display.display();
  }
}

/**
 * @brief Checks for valid GPS data, formats it, and calls the draw function.
 */
void updateDisplayData() {
  String lat_str = "---";
  String lng_str = "---";
  String time_str = "--:--:--";
  String date_str = "--/--/----";

  if (gps.location.isValid()) {
    lat_str = String(gps.location.lat(), 6);
    lng_str = String(gps.location.lng(), 6);
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    // Get UTC time components
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Apply timezone offset
    minute += TIME_ZONE_MINUTE_OFFSET;
    hour += minute / 60;
    minute %= 60;

    hour += TIME_ZONE_HOUR_OFFSET;
    hour %= 24;
    
    // Format time string with leading zeros
    char time_buffer[12];
    snprintf(time_buffer, sizeof(time_buffer), "%02d:%02d:%02d", hour, minute, second);
    time_str = time_buffer;
    
    // Format date string
    char date_buffer[12];
    snprintf(date_buffer, sizeof(date_buffer), "%02d/%02d/%d", gps.date.day(), gps.date.month(), gps.date.year());
    date_str = date_buffer;
  }

  // Combine date and time for display
  String datetime_str = date_str + " " + time_str;
  drawOLED(datetime_str, lat_str, lng_str);
}

/**
 * @brief Draws the formatted information onto the OLED display.
 * @param datetime_str The formatted date and time string.
 * @param lat_str The formatted latitude string.
 * @param lng_str The formatted longitude string.
 */
void drawOLED(const String& datetime_str, const String& lat_str, const String& lng_str) {
  display.clearDisplay(); // Korrigiert: clear() -> clearDisplay()
  
  // Adafruit GFX nutzt setCursor und print/println anstelle von setTextAlignment und drawString.
  // Die Koordinaten wurden bestmöglich den ursprünglichen Werten nachempfunden.

  // Draw the title: "GPS INFO" (Schriftgröße 2)
  display.setTextSize(2);
  display.setCursor(0, 0); 
  display.print("GPS INFO");

  // Draw Latitude (Schriftgröße 1)
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.print("Lat: " + lat_str);

  // Draw Longitude (Schriftgröße 1)
  display.setCursor(0, 30);
  display.print("Lon: " + lng_str);

  // Draw Date and Time (Schriftgröße 2)
  display.setTextSize(2);
  display.setCursor(0, 46);
  display.print(datetime_str);

  display.display(); // Write the buffer to the display
}
