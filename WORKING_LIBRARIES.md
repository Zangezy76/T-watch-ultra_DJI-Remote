# Рабочие версии библиотек для T-Watch Ultra

Проверено: октябрь 2026. Скетч `arduino/DJI_Remote_T-Watch` собирается без предупреждений
и проверен на часах (v2.1.0). До v2.0.0 включительно использовалось ядро esp32 3.3.8.

## Arduino Core
| Библиотека | Версия |
|---|---|
| esp32 by Espressif Systems | 3.3.12 |

## Основные библиотеки
| Библиотека | Версия | Источник |
|---|---|---|
| LilyGoLib | 0.1.0 | github.com/Xinyuan-LilyGO/LilyGoLib (свежая) |
| SensorLib | 0.3.3 | LilyGoLib-ThirdParty (НЕ обновлять!) |
| RadioLib | 7.4.0 | LilyGoLib-ThirdParty (НЕ обновлять!) |
| lvgl | 9.4.0 | LilyGoLib-ThirdParty (НЕ обновлять!) |
| NimBLE-Arduino | 2.5.0 | Arduino Library Manager |
| TinyGPSPlus | 1.1.0 | отдельная библиотека; класс GPS из LilyGoLib наследует TinyGPSPlus |

## NFC библиотеки
| Библиотека | Версия | Источник |
|---|---|---|
| NFC-RFAL-fork | 1.0.1 | github.com/lewisxhe/NFC-RFAL-fork |
| ST25R3916-fork | 1.1.0 | github.com/lewisxhe/ST25R3916-fork |

## Важные заметки

### Установка
1. Установить esp32 core 3.3.12 через Boards Manager
2. Клонировать LilyGoLib в Arduino/libraries/
3. Клонировать LilyGoLib-ThirdParty и скопировать ВСЕ папки в Arduino/libraries/
4. Заменить SensorLib, RadioLib, lvgl на версии из ThirdParty

### НЕ обновлять автоматически!
Arduino IDE будет предлагать обновить библиотеки — ОТКАЗЫВАТЬ.
Версии из ThirdParty специально подобраны для совместимости.

### Настройки платы
Плата **LilyGo T-Watch-Ultra**, настройки по умолчанию: USB CDC On Boot — Enabled,
USB Mode — Hardware CDC and JTAG, Upload Mode — UART0 / Hardware CDC,
Partition Scheme — 16M Flash (3MB APP/9.9MB FATFS), Board Revision — Radio-SX1262
(или реальный радиомодуль), Erase All Flash — Disabled.

### Прошивка (важно!)
- В Arduino IDE прошивка идёт автоматически без нажатий кнопок
- Из командной строки:
  `arduino-cli compile --fqbn esp32:esp32:twatch_ultra -u -p COM6 arduino/DJI_Remote_T-Watch`
- Arduino IDE может прошить устаревший текст из открытой вкладки, если файл меняли
  снаружи, — перед загрузкой переоткрыть скетч
- После прошивки по USB часы иногда стартуют до ~3 минут (периферия не обесточивается);
  при включении кнопкой — около 5 с
- Если порт не виден или «прыгает»: зажать BOOT, нажать и отпустить RST, отпустить BOOT,
  затем прошить; после прошивки нажать RST
- Запасной вариант: COM порт через Chrome WebSerial (espressif.github.io/esp-launchpad) или
  `python -m esptool --chip esp32s3 --port COM6 -b 115200 --before no_reset --after no_reset write_flash -z 0x0 [файл.bin]`
  (во время Connecting... нажать BOOT + Reset)
