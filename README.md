# Vartoviy-Device

📋 Описание кода для GPS трекера с шокером и бузером

Этот код предназначен для управления GPS трекером с шокером и бузером, используя ESP8266 и модуль A9G. Он предоставляет функционал для отслеживания местоположения, контроля геозоны, отправки данных на MQTT-брокер и управления шокером и бузером.

🔧 Настройка и прошивка

1. Установите необходимые библиотеки: ArduinoJson, EEPROM, WiFiManager, PubSubClient, Wire, Adafruit_Sensor, Adafruit_BME280, SoftwareSerial, ESP8266_Tones и Tone_Pitches.
2. Скачайте код и откройте его в Arduino IDE.
3. Настройте пины для подключения компонентов в начале кода.
4. Загрузите код на ESP8266.

🔍 Функции кода

🌐 Подключение к Wi-Fi и MQTT

Код обеспечивает подключение к Wi-Fi сети и MQTT-брокеру. При первом запуске устройства оно создает точку доступа с именем "ESP8266-Config", к которой можно подключиться для настройки параметров Wi-Fi и MQTT.

📡 Работа с GPS и A9G

Код использует модуль A9G для получения данных GPS и отправки их на MQTT-брокер. Он также предоставляет функции для управления модулем A9G, такие как включение/выключение и проверка силы сигнала.

🗺 Контроль геозоны

Код предоставляет функционал для контроля геозоны. Он хранит данные о геозоне в EEPROM и проверяет, находится ли устройство внутри или вне геозоны. Если устройство выходит из геозоны, активируется шокер.

📤 Отправка данных на MQTT-брокер

Код отправляет данные GPS, температуру, влажность, давление и уровень заряда батареи на MQTT-брокер.

🔊 Управление шокером и бузером

Код обеспечивает управление шокером и бузером. Шокер активируется, когда устройство выходит из геозоны, а бузер звучит, когда устройство приближается к границе геозоны.

🔄 Основной цикл

В основном цикле кода выполняется проверка подключения к сети, получение данных GPS, обработка кнопки, отправка данных на MQTT-брокер и проверка геозоны.

💾 Сохранение данных в EEPROM

Код сохраняет данные о геозоне и настройки Wi-Fi и MQTT в EEPROM, чтобы они не терялись при перезагрузке устройства.

🔋 Контроль заряда батареи

Код измеряет напряжение батареи и рассчитывает процент заряда. Эти данные отправляются на MQTT-брокер для мониторинга.

🔔 Музыкальные уведомления

Код предоставляет функцию playDoomMelody(), которая воспроизводит мелодию из Doom, когда устройство приближается к границе геозоны.

🔧 Обработка кнопки

Код обеспечивает обработку нажатий кнопки для управления режимами Wi-Fi, включением/выключением модуля A9G и перезагрузкой устройства.
