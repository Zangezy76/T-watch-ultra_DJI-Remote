# Рабочие версии библиотек для T-Watch Ultra

Проверено: май 2026. helloworld компилируется и прошивается успешно.

## Arduino Core
| Библиотека | Версия |
|---|---|
| esp32 by Espressif Systems | 3.3.8 |

## Основные библиотеки
| Библиотека | Версия | Источник |
|---|---|---|
| LilyGoLib | 0.1.0 | github.com/Xinyuan-LilyGO/LilyGoLib (свежая) |
| SensorLib | 0.3.3 | LilyGoLib-ThirdParty (НЕ обновлять!) |
| RadioLib | 7.4.0 | LilyGoLib-ThirdParty (НЕ обновлять!) |
| lvgl | 9.4.0 | LilyGoLib-ThirdParty (НЕ обновлять!) |

## NFC библиотеки
| Библиотека | Версия | Источник |
|---|---|---|
| NFC-RFAL-fork | 1.0.1 | github.com/lewisxhe/NFC-RFAL-fork |
| ST25R3916-fork | 1.1.0 | github.com/lewisxhe/ST25R3916-fork |

## Важные заметки

### Установка
1. Установить esp32 core 3.3.8 через Boards Manager
2. Клонировать LilyGoLib в Arduino/libraries/
3. Клонировать LilyGoLib-ThirdParty и скопировать ВСЕ папки в Arduino/libraries/
4. Заменить SensorLib, RadioLib, lvgl на версии из ThirdParty

### НЕ обновлять автоматически!
Arduino IDE будет предлагать обновить библиотеки — ОТКАЗЫВАТЬ.
Версии из ThirdParty специально подобраны для совместимости.

### Прошивка (важно!)
- COM порт виден только через Chrome WebSerial (espressif.github.io/esp-launchpad)
- Для входа в bootloader: запустить esptool, во время Connecting... нажать BOOT + Reset
- Команда: python -m esptool --chip esp32s3 --port COM6 -b 115200 --before no_reset --after no_reset write_flash -z 0x0 [файл.bin]
- В Arduino IDE прошивка идёт автоматически без нажатий кнопок
