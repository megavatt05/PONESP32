/**
 * RS485 Motor Control (ESP32)
 * ---------------------------
 * Управление двигателем через RS485/Modbus RTU.
 * Тестирование командой Serial Monitor без TFT/Keypad/Encoder.
 *
 * Библиотеки:
 * - SoftwareSerial (встроенная)
 * - iarduino_Modbus
 */

#include <SoftwareSerial.h>
#include <iarduino_Modbus.h>

// ---------- Настройки RS485 ----------
#define RXD2 16
#define TXD2 17
#define DE_RE_PIN 27

SoftwareSerial rs485(RXD2, TXD2);
ModbusClient modbus(rs485, DE_RE_PIN);

// ---------- Переменные состояния ----------
float distance = 0.0; // Текущее расстояние (эмуляция энкодера)
String value2 = "";   // Целевая длина (ввод с клавиатуры)

// ---------- Прототипы ----------
void handleKeyPress(char key);
void printHelp();

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  rs485.begin(9600);
  modbus.begin();
  modbus.setTimeout(200);
  modbus.setDelay(200);

  Serial.println();
  Serial.println("=========================================");
  Serial.println(" RS485 Motor Control - ESP32");
  Serial.println("=========================================");
  printHelp();
}

// ---------- Loop ----------
void loop() {
  if (Serial.available()) {
    char key = Serial.read();
    handleKeyPress(key);
  }
}

// ---------- Обработка клавиш ----------
void handleKeyPress(char key) {
  // Ввод цифр и точки (для дробной длины)
  if ((key >= '0' && key <= '9') || key == '.') {
    value2 += key;
    Serial.print("Целевая длина: ");
    Serial.println(value2);
    return;
  }

  switch (key) {
    case '*':
      value2 = "";
      Serial.println("Длина сброшена");
      break;

    case '+':
      distance += 0.1;
      Serial.print("Текущее расстояние: ");
      Serial.println(distance, 2);
      break;

    case '-':
      distance -= 0.1;
      if (distance < 0) distance = 0;
      Serial.print("Текущее расстояние: ");
      Serial.println(distance, 2);
      break;

    case 'D': {
      // ПУСК двигателя
      Serial.println(">> Команда ПУСК");
      if (modbus.holdingRegisterWrite(1, 8192, 18)) {
        Serial.println(" Двигатель: ПУСК OK");
      } else {
        Serial.println(" Ошибка записи регистра 8192!");
        break;
      }

      // Установка скорости разгона
      if (modbus.holdingRegisterWrite(1, 8193, 1000)) {
        Serial.println(" Скорость разгона: 1000 OK");
      } else {
        Serial.println(" Ошибка записи регистра 8193!");
      }
      break;
    }

    case '#': {
      // СТОП с проверкой достижения длины
      Serial.println(">> Команда СТОП");
      float target = value2.toFloat();
      if (distance == target) {
        if (modbus.holdingRegisterWrite(1, 8192, 1)) {
          Serial.println(" Двигатель: СТОП OK");
        } else {
          Serial.println(" Ошибка записи регистра 8192!");
        }
      } else {
        Serial.print(" Целевая длина не достигнута (текущ: ");
        Serial.print(distance, 2);
        Serial.print(" / цель: ");
        Serial.print(target, 2);
        Serial.println(")");
      }
      break;
    }

    case 'h':
    case 'H':
      printHelp();
      break;

    default:
      // Игнорируем \r, \n и прочее
      break;
  }
}

// ---------- Справка ----------
void printHelp() {
  Serial.println("-----------------------------------------");
  Serial.println("Команды:");
  Serial.println(" 0-9, . : ввод целевой длины");
  Serial.println(" *      : сброс введённой длины");
  Serial.println(" + / -  : эмуляция энкодера (+/- 0.1)");
  Serial.println(" D      : ПУСК двигателя");
  Serial.println(" #      : СТОП двигателя");
  Serial.println(" h      : эта справка");
  Serial.println("-----------------------------------------");
}
