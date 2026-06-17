# Abhängigkeiten

Reproduzierbarer Build über **PlatformIO** mit der **pioarduino**-Plattform
(Arduino-ESP32 Core 3.1.x). Alle Versionen sind in `platformio.ini` gepinnt.

| Abhängigkeit | Gepinnte Version | Zweck |
|--------------|------------------|-------|
| Plattform `pioarduino` | `53.03.13` | Arduino-ESP32 Core 3.1.x (für `ledcAttach` u.a.) |
| `bblanchon/ArduinoJson` | `^6.21.5` | JSON-Konfiguration & API |
| `knolleary/PubSubClient` | `^2.8` | MQTT-Client |
| `lovyan03/LovyanGFX` | `^1.1.16` | TFT-Display (ILI9341 / CYD) |

## Warum pioarduino?

Der Sketch verwendet die in **Arduino-ESP32 Core 3.x** geänderte LEDC-API
(`ledcAttach(pin, freq, res)` / `ledcWrite(pin, duty)`). Das offizielle
PlatformIO (`espressif32`) liefert nur Core 2.x. Der pioarduino-Fork bietet
Core 3.x; eine sehr neue Version (54/55.x) scheiterte auf diesem Setup beim
Tool-Install, daher die stabile `53.03.13`.

## Display

Display-Treiber und Pins werden **im Sketch selbst** über die LovyanGFX-
Konfigurationsklasse gesetzt (Board ESP32-2432S028 „CYD", ILI9341 + XPT2046).
Bei abweichender Hardware dort anpassen.

## WebUI-Passwort

Default `changeme`. Per Build-Flag überschreibbar (nicht im Code hinterlegen):
`-DRG_PASSWORD='"DeinSicheresPasswort"'`.
