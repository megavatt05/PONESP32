#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <Wire.h>

// ==================== Configuration ====================
#define WIFI_SSID "your_SSID"
#define WIFI_PASSWORD "your_PASSWORD"
#define I2C_KEYBOARD_ADDR 0x50

// Encoder
#define ENC_CLK 13
#define ENC_DT 14
#define ENC_SW 15

// Display (SPI)
#define TFT_CS 5
#define TFT_DC 2
#define TFT_PWR 12 // управление питанием/EN дисплея

// RS485
#define RS485_DE 4
#define RS485_RX 16
#define RS485_TX 17

// ==================== Objects ====================
Adafruit_ILI9341 tft(TFT_CS, TFT_DC);
ESP32Encoder encoder;
HardwareSerial rs485(2);

// ==================== State ====================
long encPosition = 0;
bool btnState = false;
String wifiStatus = "Disconnected";
unsigned long lastWifiUpdate = 0;
unsigned long lastEncoderDraw = 0;

// ==================== Helpers ====================
void drawStatus(const char* label, const char* value, int y) {
  tft.fillRect(0, y, 320, 24, ILI9341_BLACK);
  tft.setCursor(0, y);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print(label);
  tft.print(value);
}

void drawEncoder(long pos) {
  tft.fillRect(0, 0, 320, 24, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.print("Encoder: ");
  tft.print(pos);
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // --- Display power (simulated EN) ---
  pinMode(TFT_PWR, OUTPUT);
  digitalWrite(TFT_PWR, HIGH);
  delay(150);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.println("System booting...");

  // --- Encoder (hardware quadrature) ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(ENC_DT, ENC_CLK);
  encoder.setCount(0);
  pinMode(ENC_SW, INPUT_PULLUP);

  // --- RS485 ---
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW); // режим приёма
  rs485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

  // --- I2C ---
  Wire.begin();

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  tft.setCursor(0, 30);
  tft.print("WiFi connecting");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    tft.print(".");
  }

  wifiStatus = (WiFi.status() == WL_CONNECTED)
                   ? "WiFi: " + WiFi.localIP().toString()
                   : "WiFi: FAILED";

  tft.fillScreen(ILI9341_BLACK);
  drawEncoder(0);
}

// ==================== Loop ====================
void loop() {
  // --- Encoder ---
  long newPos = encoder.getCount();
  if (newPos != encPosition) {
    encPosition = newPos;
    drawEncoder(encPosition);
  }

  // --- Encoder button ---
  bool newBtn = !digitalRead(ENC_SW);
  if (newBtn != btnState) {
    btnState = newBtn;
    drawStatus("Button: ", btnState ? "PRESSED " : "RELEASED", 30);

    if (btnState) {
      // длинное нажатие (>1 сек) = сброс счётчика
      unsigned long t = millis();
      while (!digitalRead(ENC_SW)) {
        if (millis() - t > 1000) {
          encoder.setCount(0);
          encPosition = 0;
          drawEncoder(0);
          break;
        }
        delay(10);
      }
    }
  }

  // --- I2C keyboard ---
  Wire.requestFrom(I2C_KEYBOARD_ADDR, 1);
  if (Wire.available()) {
    char key = Wire.read();
    char buf[8];
    snprintf(buf, sizeof(buf), "'%c'", key);
    drawStatus("Key: ", buf, 60);
  }

  // --- RS485 echo ---
  if (rs485.available()) {
    String rx = rs485.readStringUntil('\n');
    rx.trim();
    digitalWrite(RS485_DE, HIGH);
    delayMicroseconds(200);
    rs485.print("Echo: ");
    rs485.println(rx);
    rs485.flush();
    delayMicroseconds(200);
    digitalWrite(RS485_DE, LOW);
  }

  // --- WiFi status ---
  if (millis() - lastWifiUpdate > 5000) {
    lastWifiUpdate = millis();
    if (WiFi.status() == WL_CONNECTED) {
      wifiStatus = "WiFi: " + WiFi.localIP().toString();
    } else {
      wifiStatus = "WiFi: DISCONNECTED";
    }
    drawStatus("", wifiStatus.c_str(), 90);
  }

  delay(10);
}
