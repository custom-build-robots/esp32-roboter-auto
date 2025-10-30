/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2025-10-30
 * Version: 2.0 
 * Description:
 * This program enables your ESP32-based robot car to be
 * controlled via a PlayStation 4/5 controller using the
 * Bluepad32 library. ENA/ENB on L298N are assumed to be
 * jumpered HIGH. Speed and direction are controlled by
 * PWM signals on IN1, IN2, IN3, IN4 pins.
 ****************************************************/

#include <WiFi.h>
#include <Adafruit_SSD1306.h> // For OLED display
#include <Bluepad32.h>   // Include Bluepad32 library
#include <Adafruit_NeoPixel.h> // For WS2812 LEDs

// ----------------------------
// Adafruit Display Definition (NEU)
// ----------------------------
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET   -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< I2C-Adresse (häufig 0x3C oder 0x3D)

// Das Display-Objekt mit der Adafruit-Klasse erstellen:
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// ----------------------------
// GPIO Pin Definitions for L298N Motor Driver INx lines
// ENA and ENB are assumed to be jumpered to HIGH externally.
// ----------------------------
#define MOTOR_LEFT_IN1_GPIO 13  // ESP32 GPIO for L298N IN1
#define MOTOR_LEFT_IN2_GPIO 12  // ESP32 GPIO for L298N IN2
#define MOTOR_RIGHT_IN3_GPIO 27 // ESP32 GPIO for L298N IN3
#define MOTOR_RIGHT_IN4_GPIO 33 // ESP32 GPIO for L298N IN4

// ----------------------------
// ESP32 LEDC PWM Configuration for INx Pins
// ----------------------------
#define MOTOR_LEFT_IN1_PWM_CHANNEL 0
#define MOTOR_LEFT_IN2_PWM_CHANNEL 1
#define MOTOR_RIGHT_IN3_PWM_CHANNEL 2
#define MOTOR_RIGHT_IN4_PWM_CHANNEL 3

#define PWM_MOTOR_FREQUENCY 5000 // Frequency in Hz for motor PWM (e.g., 5000 Hz)
#define PWM_MOTOR_RESOLUTION 8   // Resolution in bits (8-bit = 0-255 speed range)
const int MAX_MOTOR_PWM_VALUE = (1 << PWM_MOTOR_RESOLUTION) - 1; // Max PWM value (255 for 8-bit)

// ----------------------------
// Robot Name
// ----------------------------
const String ROBOT_NAME = "Ingmar's Truck";

// ----------------------------
// Motor Speeds (-100 to 100)
// ----------------------------
int speed_left = 0;
int speed_right = 0;

// ----------------------------
// Command Timeout for Safety
// ----------------------------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

// ----------------------------
// Inversion Option
// ----------------------------
const bool invert_controls = false; // Set to 'true' to invert controls

// ----------------------------
// Direction Multiplier
// ----------------------------
const int direction_multiplier = invert_controls ? -1 : 1;

// ----------------------------
// NeoPixel LED Ring Setup
// ----------------------------
#define LED_PIN 5
#define NUM_LEDS 12
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum LedMode {
  OFF_MODE,
  AMBIENT_MODE,
  POLICE_MODE,
  WHITE_MODE
};
LedMode currentLedMode = AMBIENT_MODE;

String ledModeName(LedMode mode) {
  switch (mode) {
    case OFF_MODE: return "Off";
    case AMBIENT_MODE: return "Ambient";
    case POLICE_MODE: return "Police";
    case WHITE_MODE: return "White";
    default: return "Unknown";
  }
}

const int DEADZONE = 10;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void updateDisplayStatus() {
  // Anpassung für Adafruit: clear() -> clearDisplay(), Text wird mit setCursor und print/println geschrieben
  display.clearDisplay(); 
  display.setCursor(0, 0);

  // Zeile 1: Name des Roboters
  display.print(ROBOT_NAME);

  // Zeile 2: LED Modus (Cursor auf Y=10)
  display.setCursor(0, 10);
  display.print("LED: " + ledModeName(currentLedMode));
  
  // Controller Status
  bool controllerConnected = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] && myControllers[i]->isConnected()) {
        controllerConnected = true;
        break;
    }
  }

  // Zeile 3: Controller Status (Cursor auf Y=20)
  display.setCursor(0, 20);
  if (controllerConnected){
    display.print("Ctrl Connected");
  } else {
    display.print("Ctrl Disconnected");
  }
  
  // Zeile 4: Motorgeschwindigkeiten (Cursor auf Y=30)
  display.setCursor(0, 30);
  display.print("L:" + String(speed_left) + "% R:" + String(speed_right) + "%");
  
  display.display(); // Zeigt den Puffer auf dem Display an
}

// ----------------------------
// Motor Control Function (PWM on INx pins, ENA/ENB jumpered HIGH)
// ----------------------------
void controlMotorSide(int side_index, int speed_percent) {
    // side_index: 0 for left motor, 1 for right motor
    // speed_percent: -100 (full backward) to 100 (full forward)

    uint8_t in_pin_A_pwm_channel, in_pin_B_pwm_channel;

    if (side_index == 0) { // Left Motor
        in_pin_A_pwm_channel = MOTOR_LEFT_IN1_PWM_CHANNEL; // Connected to MOTOR_LEFT_IN1_GPIO
        in_pin_B_pwm_channel = MOTOR_LEFT_IN2_PWM_CHANNEL; // Connected to MOTOR_LEFT_IN2_GPIO
    } else { // Right Motor
        in_pin_A_pwm_channel = MOTOR_RIGHT_IN3_PWM_CHANNEL; // Connected to MOTOR_RIGHT_IN3_GPIO
        in_pin_B_pwm_channel = MOTOR_RIGHT_IN4_PWM_CHANNEL; // Connected to MOTOR_RIGHT_IN4_GPIO
    }

    // Calculate PWM duty cycle from speed percentage
    int pwm_duty = map(abs(speed_percent), 0, 100, 0, MAX_MOTOR_PWM_VALUE);
    pwm_duty = constrain(pwm_duty, 0, MAX_MOTOR_PWM_VALUE); // Ensure it's within 0-MAX_MOTOR_PWM_VALUE

    if (speed_percent > 0) { // Forward for this side
        // Pin A gets PWM signal, Pin B is LOW (0% duty cycle)
        ledcWrite(in_pin_A_pwm_channel, pwm_duty);
        ledcWrite(in_pin_B_pwm_channel, 0);
    } else if (speed_percent < 0) { // Backward for this side
        // Pin A is LOW (0% duty cycle), Pin B gets PWM signal
        ledcWrite(in_pin_A_pwm_channel, 0);
        ledcWrite(in_pin_B_pwm_channel, pwm_duty);
    } else { // Stop
        // Both pins LOW (0% duty cycle) - L298N will brake with ENA/ENB HIGH
        ledcWrite(in_pin_A_pwm_channel, 0);
        ledcWrite(in_pin_B_pwm_channel, 0);
    }
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected, index=%d\n", i);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("Controller connected, but no empty slot found");
  }
  lastCommandTime = millis();
  updateDisplayStatus();
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected, index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController) {
    Serial.println("Controller disconnected, but not found in myControllers");
  }

  speed_left = 0;
  speed_right = 0;
  controlMotorSide(0, speed_left); // Stop left motor
  controlMotorSide(1, speed_right); // Stop right motor
  Serial.println("Controller disconnected. Motors stopped.");
  updateDisplayStatus();
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      if (ctl->isGamepad()) {
        processGamepad(ctl);
      }
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  int16_t axisX = ctl->axisX();
  int16_t axisY = ctl->axisY();

  int speed_input = -axisY * 100 / 512;
  int turn_input = axisX * 100 / 512;

  if (abs(speed_input) < DEADZONE) speed_input = 0;
  if (abs(turn_input) < DEADZONE) turn_input = 0;

  speed_input *= direction_multiplier;
  turn_input *= direction_multiplier;

  speed_left = speed_input + turn_input;
  speed_right = speed_input - turn_input;

  speed_left = constrain(speed_left, -100, 100);
  speed_right = constrain(speed_right, -100, 100);

  controlMotorSide(0, speed_left);
  controlMotorSide(1, speed_right);

  lastCommandTime = millis();
  updateDisplayStatus();

  if (ctl->x()) { // Square button for PS controller
    currentLedMode = (LedMode)((currentLedMode + 1) % 5);
    Serial.println("LED Mode changed to: " + ledModeName(currentLedMode));
    updateDisplayStatus();
    delay(200); // Debounce
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Initializing Robot Car Control (PWM on INx, ENA/ENB Jumpered HIGH)...");

  Serial.println("Initializing OLED Display");

  // Adafruit Display Initialisierung
  // SSD1306_SWITCHCAPVCC = erzeugt die Display-Spannung intern aus 3.3V
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
 
  // NEUE ANPASSUNG: Display um 180 Grad drehen
  display.setRotation(2); // 0 = keine Drehung, 1 = 90°, 2 = 180°, 3 = 270°
 
  // Adafruit GFX Texteinstellungen
  // NEUE ANPASSUNG: Textgröße auf die kleinste Stufe (1) setzen
  display.setTextSize(1);             // Normale 1:1 Pixel-Skalierung
  display.setTextColor(SSD1306_WHITE);        // Weißen Text zeichnen

  updateDisplayStatus(); // Initial display

  Serial.println("Initializing Motor INx pins for PWM control...");
  // Setup PWM channels
  ledcSetup(MOTOR_LEFT_IN1_PWM_CHANNEL, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  ledcSetup(MOTOR_LEFT_IN2_PWM_CHANNEL, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  ledcSetup(MOTOR_RIGHT_IN3_PWM_CHANNEL, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  ledcSetup(MOTOR_RIGHT_IN4_PWM_CHANNEL, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);

  // Attach PWM channels to the INx GPIO pins
  ledcAttachPin(MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_LEFT_IN2_GPIO, MOTOR_LEFT_IN2_PWM_CHANNEL);
  ledcAttachPin(MOTOR_RIGHT_IN3_GPIO, MOTOR_RIGHT_IN3_PWM_CHANNEL);
  ledcAttachPin(MOTOR_RIGHT_IN4_GPIO, MOTOR_RIGHT_IN4_PWM_CHANNEL);
  Serial.println("Motor INx pins initialized for PWM.");

  // Ensure motors are stopped at startup
  controlMotorSide(0, 0); // Stop left motor
  controlMotorSide(1, 0); // Stop right motor

  Serial.println("Initializing Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys(); // Uncomment for development if needed
  BP32.enableVirtualDevice(false);

  // Initialer Bildschirm (angepasst für Adafruit)
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(ROBOT_NAME);    
  display.setCursor(0, 20);
  display.print("Waiting for Controller"); 
  display.display();
  Serial.println("Setup complete. Waiting for controller...");

  strip.begin();
  strip.show();
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  // Safety: Stop motors if no commands received recently and a controller was/is active
  bool anyControllerEverActive = false;
   for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr) { // Check if a slot was ever used or is active
        anyControllerEverActive = true;
        break;
    }
  }
  // Check if a controller is currently connected
  bool controllerCurrentlyConnected = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (myControllers[i] && myControllers[i]->isConnected()){
          controllerCurrentlyConnected = true;
          break;
      }
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // Only stop if motors are running AND (no controller is connected OR a controller is connected but not sending commands)
    if ((speed_left != 0 || speed_right != 0)) {
      if (!controllerCurrentlyConnected || anyControllerEverActive) { // Timeout applies if a controller was expected or is unresponsive
        speed_left = 0;
        speed_right = 0;
        controlMotorSide(0, speed_left);
        controlMotorSide(1, speed_right);
        Serial.println("Command timeout. Motors stopped.");
        updateDisplayStatus();
      }
    }
  }
  updateLEDs();
  delay(1);
}

// LED update function (updateLEDs) remains the same as your previous version.
// For brevity, I'm not repeating it here, but it should be included in your final code.
// Make sure the updateLEDs function is present in your .ino file.
// ... (Assume updateLEDs function from your previous code is here) ...
void updateLEDs() {
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();

  switch (currentLedMode) {
    case OFF_MODE:
      strip.clear();
      strip.show();
      break;

    case AMBIENT_MODE:
      if (currentMillis - previousMillis >= 50) { // Update every 50ms
        previousMillis = currentMillis;
        static uint16_t hue = 0;
        for (int i = 0; i < 6; i++) { // Front LEDs 0-5
          uint32_t color = strip.gamma32(strip.ColorHSV(hue + (i * 65536L / 6)));
          strip.setPixelColor(i, color);
        }
        for (int i = 6; i < 12; i++) { // Back LEDs 6-11
          uint32_t color = strip.gamma32(strip.ColorHSV(hue + ((i - 6) * 65536L / 6) + 1000)); // Slight offset
          strip.setPixelColor(i, color);
        }
        strip.show();
        hue += 1536;
      }
      break;

    case POLICE_MODE:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        static bool toggle = false;
        strip.clear();
        if (!toggle) {
          for (int i = 0; i < 3; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
          for (int i = 3; i < 6; i++) strip.setPixelColor(i, strip.Color(0, 0, 255));
          for (int i = 6; i < 9; i++) strip.setPixelColor(i, strip.Color(0, 0, 255));
          for (int i = 9; i < 12; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
        } else {
          for (int i = 0; i < 3; i++) strip.setPixelColor(i, strip.Color(0, 0, 255));
          for (int i = 3; i < 6; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
          for (int i = 6; i < 9; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
          for (int i = 9; i < 12; i++) strip.setPixelColor(i, strip.Color(0, 0, 255));
        }
        strip.show();
        toggle = !toggle;
      }
      break;

    case WHITE_MODE:
      // Only set and show if it hasn't been set to white already to reduce writes
      static bool whiteModeActive = false;
      if (!whiteModeActive || currentMillis - previousMillis >= 1000) { // Update occasionally or if mode just changed
          for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, strip.Color(255, 255, 255));
          }
          strip.show();
          whiteModeActive = true;
          previousMillis = currentMillis; // For periodic update if desired
      }
      if(currentLedMode != WHITE_MODE) whiteModeActive = false; // Reset flag if mode changes
      break;
  }
}
