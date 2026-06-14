/*
 * VURP — Milestone 7: "Status screen"
 * ---------------------------------------
 * The XIAO ESP32-C3 creates its own WiFi network. The web page can turn the
 * NeoPixel ring on/off, drive the rover with an on-screen joystick (smooth,
 * proportional differential steering), and still test the two motors one at a
 * time. A little I2C OLED now shows the WiFi name, IP, and live status — no need
 * for the Serial Monitor to find the rover.
 *
 * Board: Seeed XIAO ESP32-C3   |   Tools: VS Code + PlatformIO
 *
 * WIRING:
 *
 *     Ring  VCC / "+"          -->  XIAO  3V3 pin
 *     Ring  GND / "-"          -->  XIAO  GND pin
 *     Ring  DIN (data INPUT)   -->  XIAO  D2
 *
 *     FeatherWing SDA          -->  XIAO  D4 / SDA / GPIO6
 *     FeatherWing SCL          -->  XIAO  D5 / SCL / GPIO7
 *     FeatherWing GND          -->  XIAO  GND
 *     FeatherWing logic power  -->  XIAO  3V3
 *     FeatherWing motor power  -->  separate motor battery terminals
 *
 *     Left/right motors        -->  FeatherWing M3 and M4
 *
 * Connect from the phone:
 *
 *     WiFi network: VURP
 *     Password:     letitroll
 *     Web page:     http://192.168.4.1/
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#define LED_PIN     D2
#define NUM_PIXELS  16
#define BRIGHTNESS  40

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
const uint8_t OLED_ADDRESS = 0x3C;
const unsigned long OLED_REFRESH_MS = 500;  // How often we check if the screen needs redrawing.

const char *WIFI_NAME = "VURP";
const char *WIFI_PASSWORD = "letitroll";

const uint8_t MOTOR_FEATHERWING_ADDRESS = 0x60;
const uint8_t LEFT_MOTOR_PORT = 3;
const uint8_t RIGHT_MOTOR_PORT = 4;
const uint8_t DEFAULT_MOTOR_SPEED = 210;  // 0..255. Tank treads need more torque than free wheels.
const uint8_t START_BOOST_SPEED = 255;    // Short kick to overcome static friction.
const unsigned long START_BOOST_MS = 180;
const unsigned long MOTOR_TEST_MS = 1500; // Safety: forward/reverse auto-stop after 1.5 seconds.

// Joystick driving (the /api/drive endpoint).
const int DRIVE_DEADBAND = 8;             // Tiny stick wobble near the centre = stop, don't whine.
const uint8_t MIN_MOVE_SPEED = 150;       // Just enough power to actually get the treads rolling.
const unsigned long DRIVE_WATCHDOG_MS = 400; // Deadman: stop if the phone goes quiet (lock/out of range).

Adafruit_NeoPixel ring(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MotorShield motorShield(MOTOR_FEATHERWING_ADDRESS);
Adafruit_DCMotor *leftMotor = nullptr;
Adafruit_DCMotor *rightMotor = nullptr;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WebServer server(80);

bool ringIsOn = false;
bool motorDriverFound = false;
bool oledFound = false;
String roverIp = "?";          // filled in once WiFi is up, then shown on the OLED.
String lastOledKey = "";       // what's currently on the screen, so we only redraw on change.
unsigned long lastOledCheck = 0;
String motorStatus = "motor ready?";
uint8_t motorSpeed = DEFAULT_MOTOR_SPEED;
uint16_t hueOffset = 0;
unsigned long lastAnimationFrame = 0;
unsigned long motorStopAt = 0;
unsigned long motorBoostEndsAt = 0;
unsigned long driveWatchdogAt = 0;
bool leftMotorActive = false;
bool rightMotorActive = false;

String ringStatusText() {
  return ringIsOn ? "ON" : "OFF";
}

void showRingOff() {
  ring.clear();
  ring.show();
}

void showRainbowFrame() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    uint16_t hue = hueOffset + (i * 65536L / NUM_PIXELS);
    ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(hue)));
  }

  ring.show();
  hueOffset += 256;
}

void stopMotor(Adafruit_DCMotor *motor) {
  if (motor != nullptr) {
    motor->run(RELEASE);
  }
}

void stopAllMotors() {
  stopMotor(leftMotor);
  stopMotor(rightMotor);

  motorStopAt = 0;
  motorBoostEndsAt = 0;
  driveWatchdogAt = 0;
  leftMotorActive = false;
  rightMotorActive = false;
  motorStatus = "stopped";
  Serial.println("Motors stopped");
}

// Drive one tread at a signed speed: positive = forward, negative = reverse,
// zero = coast to a stop. Used by the joystick (proportional) driving.
void setTread(Adafruit_DCMotor *motor, int signedSpeed) {
  if (motor == nullptr) {
    return;
  }

  if (signedSpeed == 0) {
    motor->run(RELEASE);
    return;
  }

  motor->setSpeed((uint8_t)constrain(abs(signedSpeed), 0, 255));
  motor->run(signedSpeed > 0 ? FORWARD : BACKWARD);
}

// Turn a joystick axis amount (-100..100) into a signed tread speed. Below the
// deadband the tread stops; above it we map smoothly up to the current top speed,
// starting at MIN_MOVE_SPEED so the treads actually move instead of just buzzing.
int treadSpeedFromInput(int input) {
  int magnitude = abs(input);

  if (magnitude < DRIVE_DEADBAND) {
    return 0;
  }

  int topSpeed = motorSpeed;
  int minSpeed = min((int)MIN_MOVE_SPEED, topSpeed);
  int speed = constrain(map(magnitude, DRIVE_DEADBAND, 100, minSpeed, topSpeed), 0, 255);

  return input > 0 ? speed : -speed;
}

void startMotorWithBoost(Adafruit_DCMotor *motor, uint8_t direction) {
  motor->setSpeed(START_BOOST_SPEED);
  motor->run(direction);
  motorBoostEndsAt = millis() + START_BOOST_MS;
}

void applyCruiseSpeed() {
  if (leftMotorActive && leftMotor != nullptr) {
    leftMotor->setSpeed(motorSpeed);
  }

  if (rightMotorActive && rightMotor != nullptr) {
    rightMotor->setSpeed(motorSpeed);
  }

  motorBoostEndsAt = 0;
}

void runMotor(Adafruit_DCMotor *motor, const char *motorName, uint8_t direction, const char *label) {
  if (!motorDriverFound || motor == nullptr) {
    motorStatus = "driver not found";
    Serial.println("Cannot run motor: Motor FeatherWing was not found on I2C.");
    return;
  }

  stopAllMotors();
  startMotorWithBoost(motor, direction);
  leftMotorActive = motor == leftMotor;
  rightMotorActive = motor == rightMotor;
  motorStopAt = millis() + MOTOR_TEST_MS;
  motorStatus = String(motorName) + " " + label + " at speed " + motorSpeed;

  Serial.print(motorName);
  Serial.print(" ");
  Serial.print(label);
  Serial.print(" at speed ");
  Serial.print(motorSpeed);
  Serial.println(" with start boost");
}

void runBothMotors(uint8_t direction, const char *label) {
  if (!motorDriverFound || leftMotor == nullptr || rightMotor == nullptr) {
    motorStatus = "driver not found";
    Serial.println("Cannot run motors: Motor FeatherWing was not found on I2C.");
    return;
  }

  stopAllMotors();
  startMotorWithBoost(leftMotor, direction);
  startMotorWithBoost(rightMotor, direction);
  leftMotorActive = true;
  rightMotorActive = true;
  motorStopAt = millis() + MOTOR_TEST_MS;
  motorStatus = String("both motors ") + label + " at speed " + motorSpeed;

  Serial.print("Both motors ");
  Serial.print(label);
  Serial.print(" at speed ");
  Serial.print(motorSpeed);
  Serial.println(" with start boost");
}

void spinMotors(uint8_t leftDirection, uint8_t rightDirection, const char *label) {
  if (!motorDriverFound || leftMotor == nullptr || rightMotor == nullptr) {
    motorStatus = "driver not found";
    Serial.println("Cannot spin motors: Motor FeatherWing was not found on I2C.");
    return;
  }

  stopAllMotors();
  startMotorWithBoost(leftMotor, leftDirection);
  startMotorWithBoost(rightMotor, rightDirection);
  leftMotorActive = true;
  rightMotorActive = true;
  motorStopAt = millis() + MOTOR_TEST_MS;
  motorStatus = String("spin ") + label + " at speed " + motorSpeed;

  Serial.print("Spin ");
  Serial.print(label);
  Serial.print(" at speed ");
  Serial.print(motorSpeed);
  Serial.println(" with start boost");
}

String jsonEscape(const String &value) {
  String escaped;

  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];

    if (c == '"' || c == '\\') {
      escaped += '\\';
    }

    escaped += c;
  }

  return escaped;
}

String statusJson() {
  return String("{") +
    "\"ringOn\":" + (ringIsOn ? "true" : "false") + "," +
    "\"motorDriverFound\":" + (motorDriverFound ? "true" : "false") + "," +
    "\"motorStatus\":\"" + jsonEscape(motorStatus) + "\"," +
    "\"motorSpeed\":" + String(motorSpeed) + "," +
    "\"startBoostSpeed\":" + String(START_BOOST_SPEED) + "," +
    "\"startBoostMs\":" + String(START_BOOST_MS) + "," +
    "\"motorPulseMs\":" + String(MOTOR_TEST_MS) +
    "}";
}

void sendStatusJson() {
  server.send(200, "application/json", statusJson());
}

String jsonStringValue(const String &body, const String &key) {
  String marker = "\"" + key + "\"";
  int keyIndex = body.indexOf(marker);

  if (keyIndex < 0) {
    return "";
  }

  int colonIndex = body.indexOf(':', keyIndex + marker.length());
  int firstQuote = body.indexOf('"', colonIndex + 1);
  int secondQuote = body.indexOf('"', firstQuote + 1);

  if (colonIndex < 0 || firstQuote < 0 || secondQuote < 0) {
    return "";
  }

  return body.substring(firstQuote + 1, secondQuote);
}

bool jsonBoolValue(const String &body, const String &key, bool fallback) {
  String marker = "\"" + key + "\"";
  int keyIndex = body.indexOf(marker);

  if (keyIndex < 0) {
    return fallback;
  }

  int colonIndex = body.indexOf(':', keyIndex + marker.length());

  if (colonIndex < 0) {
    return fallback;
  }

  String value = body.substring(colonIndex + 1);
  value.trim();
  return value.startsWith("true");
}

int jsonIntValue(const String &body, const String &key, int fallback) {
  String marker = "\"" + key + "\"";
  int keyIndex = body.indexOf(marker);

  if (keyIndex < 0) {
    return fallback;
  }

  int colonIndex = body.indexOf(':', keyIndex + marker.length());

  if (colonIndex < 0) {
    return fallback;
  }

  String value = body.substring(colonIndex + 1);
  value.trim();
  return value.toInt();
}

void handleHome() {
  File indexFile = LittleFS.open("/index.html", "r");

  if (!indexFile) {
    server.send(500, "text/plain", "Missing /index.html. Upload the LittleFS filesystem image.");
    return;
  }

  server.streamFile(indexFile, "text/html");
  indexFile.close();
}

void handleApiRing() {
  ringIsOn = jsonBoolValue(server.arg("plain"), "on", !ringIsOn);
  Serial.print("Ring is now ");
  Serial.println(ringStatusText());

  if (!ringIsOn) {
    showRingOff();
  }

  sendStatusJson();
}

void handleApiMotor() {
  String command = jsonStringValue(server.arg("plain"), "cmd");

  if (command == "both_forward") {
    runBothMotors(FORWARD, "forward");
  } else if (command == "both_reverse") {
    runBothMotors(BACKWARD, "reverse");
  } else if (command == "spin_left") {
    spinMotors(BACKWARD, FORWARD, "left");
  } else if (command == "spin_right") {
    spinMotors(FORWARD, BACKWARD, "right");
  } else if (command == "m3_forward") {
    runMotor(leftMotor, "M3", FORWARD, "forward");
  } else if (command == "m3_reverse") {
    runMotor(leftMotor, "M3", BACKWARD, "reverse");
  } else if (command == "m4_forward") {
    runMotor(rightMotor, "M4", FORWARD, "forward");
  } else if (command == "m4_reverse") {
    runMotor(rightMotor, "M4", BACKWARD, "reverse");
  } else {
    stopAllMotors();
  }

  sendStatusJson();
}

void handleApiDrive() {
  if (!motorDriverFound || leftMotor == nullptr || rightMotor == nullptr) {
    motorStatus = "driver not found";
    sendStatusJson();
    return;
  }

  String body = server.arg("plain");
  int x = constrain(jsonIntValue(body, "x", 0), -100, 100);  // turn: + = right
  int y = constrain(jsonIntValue(body, "y", 0), -100, 100);  // throttle: + = forward

  // Differential-drive mixing: combine throttle and turn into a speed per tread.
  int left = y + x;
  int right = y - x;

  // Mixing can push a tread past 100; scale both back together so we keep the
  // turn ratio instead of just clipping one side.
  int largest = max(abs(left), abs(right));
  if (largest > 100) {
    left = left * 100 / largest;
    right = right * 100 / largest;
  }

  int leftSpeed = treadSpeedFromInput(left);
  int rightSpeed = treadSpeedFromInput(right);

  setTread(leftMotor, leftSpeed);
  setTread(rightMotor, rightSpeed);

  // Joystick mode owns the motors now: drop the discrete-test timers and arm the
  // deadman watchdog instead so a quiet phone stops the rover.
  motorStopAt = 0;
  motorBoostEndsAt = 0;
  leftMotorActive = leftSpeed != 0;
  rightMotorActive = rightSpeed != 0;

  if (leftSpeed == 0 && rightSpeed == 0) {
    driveWatchdogAt = 0;
    motorStatus = "centered";
  } else {
    driveWatchdogAt = millis() + DRIVE_WATCHDOG_MS;
    motorStatus = String("drive L") + leftSpeed + " R" + rightSpeed;
  }

  sendStatusJson();
}

void handleApiSpeed() {
  int requestedSpeed = jsonIntValue(server.arg("plain"), "speed", motorSpeed);
  motorSpeed = (uint8_t)constrain(requestedSpeed, 80, 255);
  motorStatus = String("speed set to ") + motorSpeed;

  if (motorStopAt > 0 && motorBoostEndsAt == 0) {
    applyCruiseSpeed();
  }

  Serial.print("Motor speed set to ");
  Serial.println(motorSpeed);

  sendStatusJson();
}

void handleLegacyToggleRing() {
  ringIsOn = !ringIsOn;
  Serial.print("Ring is now ");
  Serial.println(ringStatusText());

  if (!ringIsOn) {
    showRingOff();
  }

  sendStatusJson();
}

void handleLegacyMotor() {
  String command = server.arg("cmd");

  if (command == "both_forward") {
    runBothMotors(FORWARD, "forward");
  } else if (command == "both_reverse") {
    runBothMotors(BACKWARD, "reverse");
  } else if (command == "spin_left") {
    spinMotors(BACKWARD, FORWARD, "left");
  } else if (command == "spin_right") {
    spinMotors(FORWARD, BACKWARD, "right");
  } else if (command == "m3_forward") {
    runMotor(leftMotor, "M3", FORWARD, "forward");
  } else if (command == "m3_reverse") {
    runMotor(leftMotor, "M3", BACKWARD, "reverse");
  } else if (command == "m4_forward") {
    runMotor(rightMotor, "M4", FORWARD, "forward");
  } else if (command == "m4_reverse") {
    runMotor(rightMotor, "M4", BACKWARD, "reverse");
  } else {
    stopAllMotors();
  }

  sendStatusJson();
}

void handleFavicon() {
  server.send(204, "text/plain", "");
}

void handleNotFound() {
  Serial.print("No web handler for ");
  Serial.print(server.method() == HTTP_GET ? "GET " : "POST ");
  Serial.println(server.uri());

  server.send(404, "text/plain", "VURP does not have that page.");
}

bool probeI2cAddress(uint8_t address) {
  Wire.setTimeOut(50);

  Serial.print("Checking I2C address 0x");
  if (address < 16) {
    Serial.print("0");
  }
  Serial.print(address, HEX);
  Serial.print(" ... ");

  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("FOUND");
    return true;
  }

  Serial.print("not found, error ");
  Serial.println(error);
  return false;
}

// Friendly name for the I2C addresses we expect on the VURP bus, so the scan
// below reads like a parts list instead of a wall of hex numbers.
const char *i2cDeviceName(uint8_t address) {
  switch (address) {
    case 0x60: return "Motor FeatherWing";
    case 0x3C: return "OLED screen";
    case 0x29: return "ToF distance sensor";
    default:   return "unknown device";
  }
}

// Walk every I2C address and print whatever answers. Great for "wire it up and
// see if it shows up" — a freshly wired OLED should appear here at 0x3C.
void scanI2cBus() {
  Serial.println("Scanning the I2C bus for devices...");
  int found = 0;

  for (uint8_t address = 0x01; address < 0x7F; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.print("  found 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.print("  <- ");
      Serial.println(i2cDeviceName(address));
    }
  }

  if (found == 0) {
    Serial.println("  ...nothing answered. Check the SDA/SCL/3V3/GND wiring.");
  } else {
    Serial.print("I2C scan done: ");
    Serial.print(found);
    Serial.println(" device(s).");
  }
}

void setupMotorDriver() {
  Wire.begin(SDA, SCL);

  Serial.print("I2C SDA pin: GPIO");
  Serial.print(SDA);
  Serial.print(" / D4, SCL pin: GPIO");
  Serial.print(SCL);
  Serial.println(" / D5");

  scanI2cBus();
  probeI2cAddress(MOTOR_FEATHERWING_ADDRESS);

  motorDriverFound = motorShield.begin();

  if (!motorDriverFound) {
    motorStatus = "driver not found";
    Serial.println("Could not find Motor FeatherWing at I2C address 0x60.");
    return;
  }

  leftMotor = motorShield.getMotor(LEFT_MOTOR_PORT);
  rightMotor = motorShield.getMotor(RIGHT_MOTOR_PORT);
  stopAllMotors();
  motorStatus = "ready on M3 and M4";
  Serial.println("Motor FeatherWing found. Ready to test M3 and M4.");
}

void setupOled() {
  // Shares the I2C bus already started by setupMotorDriver().
  oledFound = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);

  if (!oledFound) {
    Serial.println("OLED not found at 0x3C. Check wiring (or it may be an SH1106).");
    return;
  }

  // Crank the contrast register to its maximum so the screen is as bright as it goes.
  oled.ssd1306_command(SSD1306_SETCONTRAST);
  oled.ssd1306_command(0xFF);

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.display();
  Serial.println("OLED ready (max brightness).");
}

// One coarse word for how the rover is moving. We deliberately keep this simple
// (not the live L/R speeds) so the screen only redraws on real changes, never
// mid-drive — that keeps the joystick feeling snappy.
const char *motionWord() {
  return (leftMotorActive || rightMotorActive) ? "moving" : "idle";
}

// A short signature of everything shown on screen. If it hasn't changed, there's
// no reason to redraw.
String oledStateKey() {
  return roverIp + "|" + (ringIsOn ? "1" : "0") + "|" + motorSpeed + "|" + motionWord();
}

void drawOledStatus() {
  if (!oledFound) {
    return;
  }

  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.println("VURP");

  oled.setTextSize(1);
  oled.setCursor(0, 20);
  oled.print("net: ");
  oled.println(WIFI_NAME);
  oled.print("ip:  ");
  oled.println(roverIp);
  oled.print("ring:");
  oled.print(ringIsOn ? "ON " : "OFF");
  oled.print("  spd:");
  oled.println(motorSpeed);
  oled.print("state: ");
  oled.println(motionWord());

  oled.display();
}

// Redraw the screen when its contents change (checked a couple of times a second).
void updateOled() {
  if (!oledFound || millis() - lastOledCheck < OLED_REFRESH_MS) {
    return;
  }

  lastOledCheck = millis();
  String key = oledStateKey();

  if (key != lastOledKey) {
    lastOledKey = key;
    drawOledStatus();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  ring.begin();
  ring.setBrightness(BRIGHTNESS);
  showRingOff();

  Serial.println("Hello from VURP! Starting WiFi motor test page...");

  if (!LittleFS.begin()) {
    Serial.println("LittleFS failed to mount. Upload the filesystem image for data/index.html.");
  } else if (!LittleFS.exists("/index.html")) {
    Serial.println("LittleFS mounted, but /index.html is missing.");
  } else {
    Serial.println("LittleFS mounted. Web page file found.");
  }

  setupMotorDriver();
  setupOled();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_NAME, WIFI_PASSWORD);

  IPAddress ip = WiFi.softAPIP();
  roverIp = ip.toString();
  Serial.print("Connect to WiFi network: ");
  Serial.println(WIFI_NAME);
  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("Open this page: http://");
  Serial.println(ip);

  drawOledStatus();   // first paint now that we know the IP

  server.on("/", HTTP_GET, handleHome);
  server.on("/api/status", HTTP_GET, sendStatusJson);
  server.on("/api/ring", HTTP_POST, handleApiRing);
  server.on("/api/motor", HTTP_POST, handleApiMotor);
  server.on("/api/drive", HTTP_POST, handleApiDrive);
  server.on("/api/speed", HTTP_POST, handleApiSpeed);
  server.on("/toggle-ring", HTTP_POST, handleLegacyToggleRing);
  server.on("/toggle", HTTP_POST, handleLegacyToggleRing);
  server.on("/motor", HTTP_POST, handleLegacyMotor);
  server.on("/favicon.ico", HTTP_GET, handleFavicon);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  server.handleClient();

  if (ringIsOn && millis() - lastAnimationFrame >= 20) {
    lastAnimationFrame = millis();
    showRainbowFrame();
  }

  if (motorBoostEndsAt > 0 && millis() >= motorBoostEndsAt) {
    applyCruiseSpeed();
  }

  if (motorStopAt > 0 && millis() >= motorStopAt) {
    stopAllMotors();
  }

  // Deadman: if the joystick has gone quiet, stop so a locked phone or a
  // dropped WiFi connection can't leave the rover driving off on its own.
  if (driveWatchdogAt > 0 && millis() >= driveWatchdogAt) {
    Serial.println("Drive watchdog: no joystick update, stopping.");
    stopAllMotors();
  }

  updateOled();
}
