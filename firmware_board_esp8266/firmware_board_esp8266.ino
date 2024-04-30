/*
Этот код реализует устройство для контроля геозоны,
которое собирает данные с датчиков и отправляет их на MQTT-брокер.
Устройство также оснащено шокером и бузером, которые активируются
в зависимости от расположения в геозоне.

Алгоритм работы кода:
1. Инициализация.
2. Основной цикл.
  2.1. Подключение к сети.
  2.2. Получение силы сигнала и данных GPS.
  2.3. Восстановление данных геозоны из EEPROM.
  2.4. Обработка кнопки.
    2.4.1. Одиночное нажатие: переключение режимов Wi-Fi STA Station (STA) Mode и Access Point (AP) Mode для настройки Wi-Fi и MQTT.
    2.4.2. Двойное нажатие: выключение/включение Wi-Fi.
    2.4.3. Тройное нажатие: включение и выключение модуля A9G.
    2.4.4. Длинное нажатие на 5 секунд: выключение/включение устройства.
  2.5. Чтение данных с датчика BMP280.
  2.6. Отправка данных GPS на MQTT.
  2.7. Проверка устройства в геозоне.
  2.8. Проверка приближения к краю геозоны.
  2.9. Активация шокера.
  2.10. Активация бузера.
  2.11. Пауза.
*/

#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <ESP8266_Tones.h>
#include <Tone_Pitches.h>

// Пины для подключения компонентов
const int BUTTON_PIN = 9; // GPIO9 - подключение кнопки
const int A9G_RX_PIN = 14; // GPIO14 - подключение к A9G TXD
const int A9G_TX_PIN = 12; // GPIO12 - подключение к A9G RXD
const int A9G_PWRKEY_PIN = 16; // GPIO16 - подключение к PWRKEY A9G
const int A9G_RESET_PIN = 15; // GPIO15 - подключение к RESET A9G
#define SHOCKER_PIN 4  // Пин для управления шокером
#define BUZZER_PIN 5  // Пин для подключения бузера

// Объекты для работы с компонентами
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_BME280 bme; // I2C
SoftwareSerial A9GSerial(A9G_RX_PIN, A9G_TX_PIN);

// Данные для подключения к брокеру MQTT
char mqttBroker[64];
char mqttUsername[32];
char mqttPassword[32];
int mqttPort;

// Имя устройства
char deviceId[32];

const int EEPROM_SIZE = 512; // Размер EEPROM для хранения данных о геозоне

// Добавление константы SEALEVELPRESSURE_HPA
#define SEALEVELPRESSURE_HPA (1013.25)
#define DOUBLE_PRESS_INTERVAL 1000  // Интервал в миллисекундах для обнаружения двойного нажатия

// Замените эти значения на фактические пины, которые вы используете
const int BATTERY_PIN = A0;
const float VOLTAGE_DIVIDER_RATIO = 2.0;  // Поменяйте на ваше реальное соотношение делителя напряжения

// Константы для калибровки и вычисления процента заряда
const float BATTERY_CALIBRATION_CONSTANT = 0.0;  // Поправка для калибровки
const int BATTERY_MIN_VALUE = 0;  // Минимальное значение измерения батареи
const int BATTERY_MAX_VALUE = 1023;  // Максимальное значение измерения батареи

// Геозона получениие GeoJson данных о геозоне в EEPROM
void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, (String(deviceId) + "/new_geozone").c_str()) == 0) {
    // Парсинг данных о геозоне
    DynamicJsonDocument doc(EEPROM_SIZE);
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.println("Failed to parse JSON");
      return;
    }

    // Сохранение данных о геозоне в EEPROM
    String geozoneData = "";
    serializeJson(doc, geozoneData);

    EEPROM.begin(EEPROM_SIZE);
    for (int i = 0; i < geozoneData.length(); ++i) {
      EEPROM.write(i, geozoneData[i]);
    }
    EEPROM.commit();
    EEPROM.end();

    Serial.println("Geozone data updated and saved to EEPROM");
  }
}

class A9GModule {
public:
  void begin() {
    A9GSerial.begin(115200);
    delay(1000);
    sendCommand("AT");
    delay(1000);
  }

  String sendCommand(String cmd, int timeout = 1000) {
    A9GSerial.println(cmd);
    delay(timeout);
    String response = "";
    while (A9GSerial.available()) {
      response += (char)A9GSerial.read();
    }
    return response;
  }

  bool connectToNetwork() {
    // Проверка статуса сети
    String response = sendCommand("AT+CGATT?");
    if (response.indexOf("1") == -1) {
      // Если не подключены, попытка подключения
      sendCommand("AT+CGATT=1");
      delay(1000);
      response = sendCommand("AT+CGATT?");
      return (response.indexOf("1") != -1);
    }
    return true; // Уже подключены
  }

  String getSignalStrength() {
    return sendCommand("AT+CSQ");
  }

  String getGPSData() {
    return sendCommand("AT+GPSRD=1");
  }
};

A9GModule a9gModule;

// Дебаунсер для кнопки
int lastButtonState = HIGH;
int buttonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long lastDoublePressTime = 0;
bool a9gDoublePressHandled = true;
int triplePressCount = 0;
bool doublePressHandled = true;

// Режимы работы
enum WifiMode {
  WIFI_STA_MODE,
  TEST_WIFI_MODE,
  CELLULAR_MODE  // Для будущего подключения через сотовую связь
};
WifiMode wifiMode = WIFI_STA_MODE;

void setup() {
  // Инициализация сериал порта
  Serial.begin(115200);

  // Инициализация кнопки
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Подключение к Wi-Fi и MQTT
  connectToWiFi();
  connectToMQTT();

  // Инициализация датчика BMP280
  bool status = bme.begin(0x77); // Используйте 0x76 или 0x77 в зависимости от адреса вашего датчика
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(A9G_PWRKEY_PIN, OUTPUT);
  pinMode(A9G_RESET_PIN, OUTPUT);

  a9gModule.begin();

  // Включение A9G
  digitalWrite(A9G_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(A9G_PWRKEY_PIN, LOW);
  delay(10000); // Пауза для стабилизации

  // Восстановление данных о геозоне из EEPROM
  restoreGeozoneData();
}

void loop() {
  // Основной цикл программы
  // Подключение к сети
  if (a9gModule.connectToNetwork()) {
    Serial.println("Connected to the network!");
  } else {
    Serial.println("Failed to connect to the network.");
  }

  // Получение силы сигнала
  String signalStrength = a9gModule.getSignalStrength();
  Serial.println("Signal Strength: " + signalStrength);

  // Получение данных GPS
  String gpsData = a9gModule.getGPSData();
  Serial.println("GPS Data: " + gpsData);

  // Восстановление данных о геозоне из EEPROM
  EEPROM.begin(EEPROM_SIZE);
  String storedGeozoneData = "";
  for (int i = 0; i < EEPROM_SIZE; ++i) {
    storedGeozoneData += char(EEPROM.read(i));
  }
  EEPROM.end();

  // Обработка данных GPS
  // Здесь предполагается, что данные GPS имеют формат, например: "12.345678,45.678901"
  int commaIndex = gpsData.indexOf(',');
  String latitudeStr = gpsData.substring(0, commaIndex);
  String longitudeStr = gpsData.substring(commaIndex + 1);

  // Преобразование строк в значения с плавающей точкой
  float latitude = latitudeStr.toFloat();
  float longitude = longitudeStr.toFloat();

  // Обработка данных, отправка на MQTT, контроль зоны и т.д.
  readBME280();

  // Обработка кнопки
  handleButton();
  handleA9GButton();
  // Заряд батареии
  readBatteryVoltage();
  // Отправка данных GPS на MQTT
  sendGPSDataToMQTT();
  // Вызов функции checkDeviceInPolygon для проверки в зоне или нет
  checkDeviceInPolygon(latitude, longitude, storedGeozoneData);
  // Вызов функции checkApproachingEdge для проверки приближения к краю зоны
  checkApproachingEdge(latitude, longitude, storedGeozoneData);

  // Ваш дополнительный код

  delay(8000); // Пауза между итерациями
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);

  // Обработка дребезга контакта кнопки
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // Обработка нажатия кнопки
      if (buttonState == LOW) {
        // Режим конфигурации Wi-Fi
        if (wifiMode == WIFI_STA_MODE) {
          Serial.println("Entering configuration mode. Connect to ESP8266-Config WiFi network.");
          WiFiManager wifiManager;
          wifiManager.resetSettings();  // Очистка предыдущих настроек

          // Данные для конфигурации Wi-Fi
          WiFiManagerParameter custom_mqtt_broker("mqtt_broker", "MQTT Broker", mqttBroker, 64);
          WiFiManagerParameter custom_mqtt_username("mqtt_username", "MQTT Username", mqttUsername, 32);
          WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT Password", mqttPassword, 32);
          WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", String(mqttPort).c_str(), 4);
          WiFiManagerParameter custom_device_id("device_id", "Device ID", deviceId, 32);

          // Добавление параметров в WiFiManager
          wifiManager.addParameter(&custom_mqtt_broker);
          wifiManager.addParameter(&custom_mqtt_username);
          wifiManager.addParameter(&custom_mqtt_password);
          wifiManager.addParameter(&custom_mqtt_port);
          wifiManager.addParameter(&custom_device_id);

          if (wifiManager.startConfigPortal("ESP8266-Config")) {
            // После успешной конфигурации
            Serial.println("Connected to WiFi. Saving configuration.");
            saveConfigToEEPROM(custom_mqtt_broker, 0, 64);
            saveConfigToEEPROM(custom_mqtt_username, 64, 32);
            saveConfigToEEPROM(custom_mqtt_password, 96, 32);
            saveConfigToEEPROM(custom_mqtt_port, 128, 4);
            saveConfigToEEPROM(custom_device_id, 132, 32);

            Serial.println("Configuration saved. Restarting.");
            ESP.restart();
            delay(5000);
          } else {
            Serial.println("Failed to connect to WiFi and hit timeout");
          }
        } else if (wifiMode == TEST_WIFI_MODE) {
          // Режим тестирования Wi-Fi
          Serial.println("Testing WiFi connection...");
          if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi not connected. Connecting...");
            connectToWiFi();
          } else {
            Serial.println("WiFi already connected.");
          }
        }
      }
    }
  }

  // Обработка двойного нажатия кнопки
  if (buttonState == HIGH && wifiMode != CELLULAR_MODE && doublePressHandled) {
    if (millis() - lastDebounceTime < DOUBLE_PRESS_INTERVAL) {
      // Обработка двойного нажатия
      doublePressHandled = false;
      if (wifiMode == WIFI_STA_MODE) {
        Serial.println("Switching to Test WiFi Mode...");
        wifiMode = TEST_WIFI_MODE;
      } else if (wifiMode == TEST_WIFI_MODE) {
        Serial.println("Switching to Configuration WiFi Mode...");
        wifiMode = WIFI_STA_MODE;
      }
    }
  }

  // Сбросить обработку двойного нажатия через определенный интервал
  if (millis() - lastDoublePressTime > DOUBLE_PRESS_INTERVAL) {
    doublePressHandled = true;
  }

  lastButtonState = reading;
}

void handleA9GButton() {
  int reading = digitalRead(BUTTON_PIN);

  // Обработка дребезга контакта кнопки
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // Обработка нажатия кнопки
      if (buttonState == LOW) {
        // Режим управления A9G
        Serial.println("Button pressed.");

        // Проверка, был ли произведен тройной щелчок
        if (triplePressCount == 2) {
          triplePressCount = 0;
          // Ваш код для включения/выключения A9G
          toggleA9GModule();
        } else {
          // Иначе, увеличиваем счетчик тройных щелчков
          triplePressCount++;
        }
      }
    }
  }

  // Сброс обработки двойного нажатия через определенный промежуток времени
  if (millis() - lastDoublePressTime > DOUBLE_PRESS_INTERVAL) {
    a9gDoublePressHandled = true;
  }

  lastButtonState = reading;
}

void toggleA9GModule() {
  digitalWrite(A9G_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(A9G_PWRKEY_PIN, LOW);
  Serial.println("A9G toggled.");
}

void connectToWiFi() {
  WiFiManager wifiManager;

  // Данные для конфигурации Wi-Fi
  WiFiManagerParameter custom_mqtt_broker("mqtt_broker", "MQTT Broker", mqttBroker, 64);
  WiFiManagerParameter custom_mqtt_username("mqtt_username", "MQTT Username", mqttUsername, 32);
  WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT Password", mqttPassword, 32);
  WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", String(mqttPort).c_str(), 4);
  WiFiManagerParameter custom_device_id("device_id", "Device ID", deviceId, 32);

  // Добавление параметров в WiFiManager
  wifiManager.addParameter(&custom_mqtt_broker);
  wifiManager.addParameter(&custom_mqtt_username);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_device_id);

  // Попытка подключения к Wi-Fi
  if (!wifiManager.autoConnect("ESP8266-Config")) {
    Serial.println("Failed to connect to WiFi and hit timeout");
    delay(3000);
    ESP.restart();
    delay(5000);
  }

  // Clear EEPROM
  for (int i = 0; i < EEPROM.length(); ++i) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();

  // Сохранение данных для подключения к брокеру в EEPROM
  saveConfigToEEPROM(custom_mqtt_broker, 0, 64);
  saveConfigToEEPROM(custom_mqtt_username, 64, 32);
  saveConfigToEEPROM(custom_mqtt_password, 96, 32);
  saveConfigToEEPROM(custom_mqtt_port, 128, 4);
  saveConfigToEEPROM(custom_device_id, 132, 32);

  wifiMode = WIFI_STA_MODE;  // Переключение в режим конфигурации Wi-Fi
}

void saveConfigToEEPROM(WiFiManagerParameter parameter, int offset, int length) {
  String value = parameter.getValue();
  for (int i = 0; i < length; ++i) {
    if (i < value.length()) {
      EEPROM.write(offset + i, value[i]);
    } else {
      EEPROM.write(offset + i, 0);
    }
  }
  EEPROM.commit();
}

void connectToMQTT() {
  // Чтение ID устройства из EEPROM
  EEPROM.begin(512);
  EEPROM.get(0, deviceId);

  // Подключение к брокеру MQTT
  mqttClient.setServer(mqttBroker, mqttPort);

  // Попытка подключения к брокеру
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");

    if (mqttClient.connect(deviceId, mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
      mqttClient.subscribe((String(deviceId) + "/new_geozone").c_str());
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

void readBME280() {
  float temperatureCelsius = bme.readTemperature();
  float temperatureFahrenheit = (temperatureCelsius * 9.0 / 5.0) + 32.0;
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // Отправка данных на MQTT-брокер
  char temperatureCelsiusStr[10];
  char temperatureFahrenheitStr[10];
  char humidityStr[10];
  char pressureStr[10];
  char altitudeStr[10];

  // Преобразование температур в строки
  dtostrf(temperatureCelsius, 6, 2, temperatureCelsiusStr);
  dtostrf(temperatureFahrenheit, 6, 2, temperatureFahrenheitStr);
  dtostrf(humidity, 6, 2, humidityStr);
  dtostrf(pressure, 6, 2, pressureStr);
  dtostrf(altitude, 6, 2, altitudeStr);

  // Преобразование массивов символов в строки
  String temperatureCelsiusTopic = String(deviceId) + "/temperatureCelsius";
  String temperatureFahrenheitTopic = String(deviceId) + "/temperatureFahrenheit";
  String humidityTopic = String(deviceId) + "/humidity";
  String pressureTopic = String(deviceId) + "/pressure";
  String altitudeTopic = String(deviceId) + "/altitude";

  // Публикация данных
  mqttClient.publish(temperatureCelsiusTopic.c_str(), temperatureCelsiusStr);
  mqttClient.publish(temperatureFahrenheitTopic.c_str(), temperatureFahrenheitStr);
  mqttClient.publish(humidityTopic.c_str(), humidityStr);
  mqttClient.publish(pressureTopic.c_str(), pressureStr);
  mqttClient.publish(altitudeTopic.c_str(), altitudeStr);
}

void readBatteryVoltage() {
  // Измерение напряжения батареи
  int sensorValue = analogRead(BATTERY_PIN);

  // Преобразование значений аналогового сигнала в напряжение с учетом калибровки
  float voltage = (sensorValue + BATTERY_CALIBRATION_CONSTANT) * (3.3 / 1023.0) * VOLTAGE_DIVIDER_RATIO;

  // Рассчитываем процент заряда батареи
  int batteryPercentage = map(sensorValue, BATTERY_MIN_VALUE, BATTERY_MAX_VALUE, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);  // Ограничиваем значения от 0 до 100

  // Публикация данных о заряде батареи и проценте на тему "DeviceID/battery"
  String batteryTopic = String(deviceId) + "/battery";
  String batteryPayload = String(voltage, 2);  // Округляем до двух знаков после запятой
  mqttClient.publish(batteryTopic.c_str(), batteryPayload.c_str());

  String batteryPercentageTopic = String(deviceId) + "/batteryPercentage";
  String batteryPercentagePayload = String(batteryPercentage);
  mqttClient.publish(batteryPercentageTopic.c_str(), batteryPercentagePayload.c_str());

  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("Battery Percentage: ");
  Serial.print(batteryPercentage);
  Serial.println("%");
}

void sendGPSDataToMQTT() {
  // Получение данных GPS
  String gpsData = a9gModule.getGPSData();

  // Обработка данных GPS
  // Здесь предполагается, что данные GPS имеют формат, например: "12.345678,45.678901"
  int commaIndex = gpsData.indexOf(',');
  String latitudeStr = gpsData.substring(0, commaIndex);
  String longitudeStr = gpsData.substring(commaIndex + 1);

  // Отправка данных GPS на MQTT
  String gpsTopic = String(deviceId) + "/gps";
  String gpsPayload = "Latitude:" + latitudeStr + ",Longitude:" + longitudeStr;

  mqttClient.publish(gpsTopic.c_str(), gpsPayload.c_str());

  Serial.println("GPS Data Sent to MQTT");
}

void restoreGeozoneData() {
  EEPROM.begin(EEPROM_SIZE);

  // Чтение данных о геозоне из EEPROM
  String storedGeozoneData = "";
  for (int i = 0; i < EEPROM_SIZE; ++i) {
    storedGeozoneData += char(EEPROM.read(i));
  }

  EEPROM.end();

  // Восстановление данных о геозоне
  if (storedGeozoneData.length() > 0) {
    // Парсинг данных о геозоне
    DynamicJsonDocument doc(EEPROM_SIZE);
    DeserializationError error = deserializeJson(doc, storedGeozoneData);

    if (error) {
      Serial.println("Failed to parse stored JSON");
      return;
    }

    // Ваш код для обработки восстановленных данных о геозоне
    Serial.println("Restored Geozone Data:");
    serializeJsonPretty(doc, Serial);
    Serial.println();
  } else {
    Serial.println("No stored Geozone Data found in EEPROM");
  }
}

void checkDeviceInPolygon(float latitude, float longitude, const String& storedGeozoneData) {
  DynamicJsonDocument geozoneDoc(EEPROM_SIZE);
  DeserializationError error = deserializeJson(geozoneDoc, storedGeozoneData);

  if (error) {
    Serial.println("Failed to parse stored JSON");
    return;
  }

  JsonArray coordinates = geozoneDoc["features"][0]["geometry"]["coordinates"][0];

  bool isInsidePolygon = isPointInPolygon(latitude, longitude, coordinates);

  if (isInsidePolygon) {
    Serial.println("Устройство в зоне");
    // Отправка данных по MQTT о том, что устройство в зоне
    String zoneStatusTopic = String(deviceId) + "/zone_status";
    String zoneStatusPayload = "In Zone";
    mqttClient.publish(zoneStatusTopic.c_str(), zoneStatusPayload.c_str());
  } else {
    Serial.println("Устройство вне зоны");
    // Отправка данных по MQTT о том, что устройство вне зоны
    String zoneStatusTopic = String(deviceId) + "/zone_status";
    String zoneStatusPayload = "Out of Zone";
    mqttClient.publish(zoneStatusTopic.c_str(), zoneStatusPayload.c_str());

    // Вызов функции activateShocker при выходе из зоны
    activateShocker();
  }
}

bool isPointInPolygon(float latitude, float longitude, JsonArray& polygon) {
  int i, j;
  bool isInside = false;

  int numVertices = polygon.size();
  for (i = 0, j = numVertices - 1; i < numVertices; j = i++) {
    float xi = polygon[i][1];
    float yi = polygon[i][0];
    float xj = polygon[j][1];
    float yj = polygon[j][0];

    bool condition1 = (yi > latitude) != (yj > latitude);
    bool condition2 = (longitude < (xj - xi) * (latitude - yi) / (yj - yi) + xi);

    if (condition1 && condition2) {
      isInside = !isInside;
    }
  }

  return isInside;
}

void checkApproachingEdge(float latitude, float longitude, const String& storedGeozoneData) {
  DynamicJsonDocument geozoneDoc(EEPROM_SIZE);
  DeserializationError error = deserializeJson(geozoneDoc, storedGeozoneData);

  if (error) {
    Serial.println("Failed to parse stored JSON");
    return;
  }

  JsonArray coordinates = geozoneDoc["features"][0]["geometry"]["coordinates"][0];

  // Предположим, что isPointInPolygon возвращает расстояние до границы зоны
  float distanceToEdge = isPointInPolygon(latitude, longitude, coordinates);

  // Если расстояние до границы менее 2 метров, вызываем activateBuzzer
  if (distanceToEdge < 2.0) {

    playDoomMelody();
    // activateBuzzer();
  }
}

unsigned long lastShockerActivationTime = 0;
int shockerActivationCount = 0;
bool shockerCooldown = false;

void activateShocker() {
  if (shockerCooldown) {
    Serial.println("Shocker in cooldown. Ignoring activation.");
    return;
  }

  unsigned long currentTime = millis();

  if (shockerActivationCount >= 5) {
    // Если уже было 5 активаций, входим в режим ожидания
    shockerCooldown = true;
    Serial.println("Shocker cooldown activated. Ignoring activations for 5 minutes.");
    delay(300000);  // Ожидание 5 минут
    shockerCooldown = false;
    shockerActivationCount = 0;
    return;
  }

  if (shockerActivationCount > 0 && currentTime - lastShockerActivationTime < 10000) {
    // Если уже была хотя бы одна активация и прошло менее 10 секунд, делаем паузу
    Serial.println("Pause for 10 seconds.");
    delay(10000);
  }

  // Активация шокера
  pinMode(SHOCKER_PIN, OUTPUT);
  digitalWrite(SHOCKER_PIN, HIGH);
  delay(1000);
  digitalWrite(SHOCKER_PIN, LOW);
  pinMode(SHOCKER_PIN, INPUT);  // Отключение вывода шокера

  Serial.println("Shocker activated!");

  shockerActivationCount++;
  lastShockerActivationTime = currentTime;
}

//void activateBuzzer() { // Если хотите просто писк то можете расскоментировать эту чать и закоментировать playDoomMelody
  // Активация бузера
//  tone(BUZZER_PIN, 1000);  // Пример: звук на частоте 1000 Гц
//  delay(500);  // Пример: активация бузера на полсекунды
//  noTone(BUZZER_PIN);  // Отключение бузера

//  Serial.println("Buzzer activated!");
//}

// Музыка уведомления
void playDoomMelody() {
  int octave = 3;
  int speed = 64;  // Измените, если необходимо

  // Быстрая часть
  for (int i = 0; i < 3; ++i) {
    tone(BUZZER_PIN, Note_B4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_G4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_E4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_C4, speed);
    delay(speed);
  }

  // Главная тема
  speed = 128;  // Измените, если необходимо

  for (int i = 0; i < 4; ++i) {
    tone(BUZZER_PIN, Note_E4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_D4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_C4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_AS4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_B4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_C4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_E4, speed);
    delay(speed);
    tone(BUZZER_PIN, Note_D4, speed);
    delay(speed);
  }

  // Дополнительное примечание
  tone(BUZZER_PIN, Note_AS4, speed * 2);
  delay(speed * 2);

  noTone(BUZZER_PIN);  // Отключение бузера

  delay(4000); // Пауза 4 секунд перед повторением
}
