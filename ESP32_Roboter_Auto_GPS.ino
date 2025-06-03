// Autor: Ingmar Stapel
// Homepage: https://custom-build-robots.com/
// Date: 2020-04-19
//
// Rewritten for ESP32 with improvements by Gemini
// - Replaced SoftwareSerial with HardwareSerial for better stability on ESP32.
// - Restructured the main loop for more efficient display updates.
// - Added a check for missing GPS signal to aid in debugging.
// - Centralized pin configuration.

#include <TinyGPS++.h>
#include <Wire.h>
#include "SSD1306Wire.h" // User's specified library for the OLED display

// --- PIN Configuration ---
// Default I2C pins for the OLED display on most ESP32 boards are:
#define OLED_SDA 21
#define OLED_SCL 22

// Define the RX and TX pins for the GPS module connected to Serial2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// --- GPS and Display Objects ---
TinyGPSPlus gps;
// Use HardwareSerial for the GPS module. Serial2 is a common choice (UART2).
HardwareSerial gpsSerial(2);
// Initialize the OLED display using the specified I2C address and pins
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

// --- Global Variables ---
// Set your local time zone offset from UTC
const int TIME_ZONE_HOUR_OFFSET = 2; // e.g., 2 for CEST
const int TIME_ZONE_MINUTE_OFFSET = 0;

// --- Function Prototypes ---
void updateDisplayData();
void drawOLED(const String& time_str, const String& lat_str, const String& lng_str);

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(115200);
  Serial.println("GPS OLED Display starting up...");

  // Start the hardware serial port for the GPS module
  // The begin method takes (baud_rate, protocol, RX_PIN, TX_PIN)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Serial connection started on pins RX:16, TX:17");

  // Initialize the OLED display
  display.init();
  display.flipScreenVertically();
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
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "No GPS data");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 24, "Check wiring to");
    display.drawString(0, 36, "GPS module.");
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
  display.clear(); // Clear the display buffer
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  // Draw the title
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "GPS INFO");

  // Draw Latitude
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 18, "Lat: " + lat_str);

  // Draw Longitude
  display.drawString(0, 30, "Lon: " + lng_str);

  // Draw Date and Time
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 46, datetime_str);

  display.display(); // Write the buffer to the display
}