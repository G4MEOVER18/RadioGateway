<div align="center">

# RadioGateway

[![CI](https://github.com/G4MEOVER18/RadioGateway/actions/workflows/platformio-build.yml/badge.svg)](https://github.com/G4MEOVER18/RadioGateway/actions/workflows/platformio-build.yml)

**ESP32-Gateway mit WiFi (AP/STA), WebUI, MQTT, SD-Konfiguration und optionalem TFT-Display.**

[![Plattform](https://img.shields.io/badge/Plattform-ESP32-blue?style=flat-square)](.)
[![Build](https://img.shields.io/badge/Build-PlatformIO%20(pioarduino)-orange?style=flat-square&logo=platformio)](https://platformio.org)
[![Core](https://img.shields.io/badge/Arduino--ESP32-3.1.x-green?style=flat-square)](.)
[![Lizenz](https://img.shields.io/badge/Lizenz-MIT-green?style=flat-square)](LICENSE)

</div>

---

Ein eigenständiges ESP32-Gateway: spannt einen Access-Point auf **und** verbindet sich mit dem WLAN (AP/STA), bietet eine passwortgeschützte **WebUI** zur Konfiguration, einen **MQTT**-Client zur Anbindung an Smart-Home/Broker, lädt seine Konfiguration von **SD-Karte** und zeigt den Status optional auf einem **TFT-Display** (LovyanGFX).

## Features

| Bereich | Funktion |
|---------|----------|
| **WiFi** | Gleichzeitig AP + STA, mDNS, Status-Anzeige |
| **WebUI** | Passwortgeschützt (BasicAuth), Status, Konfiguration, REST-API (`/api/status`, `/api/clients`, …) |
| **MQTT** | PubSubClient, konfigurierbarer Broker, Last-Message-Anzeige |
| **SD** | Konfiguration laden/speichern, Diagnose |
| **Display** | Optional ILI9341/CYD über LovyanGFX, Backlight-PWM (Core-3.x-LEDC) |

## ⚠️ Vor dem ersten Einsatz

Das WebUI-Passwort hat den Default `changeme` — **unbedingt ändern**, am besten per Build-Flag (nicht im Code):

```ini
build_flags = -DRG_PASSWORD='"DeinSicheresPasswort"'
```

## Fertige Firmware

Vorkompiliertes Image unter [**Releases**](https://github.com/G4MEOVER18/RadioGateway/releases/latest):

```bash
esptool.py --chip esp32 write_flash 0x0 RadioGateway_esp32_v1.0.0_merged.bin
```

## Selbst bauen (PlatformIO)

```bash
pio run                       # bauen
pio run -t upload             # flashen
pio device monitor -b 115200  # serielle Ausgabe
```

Der Sketch nutzt die **Arduino-ESP32 Core 3.1.x LEDC-API** (`ledcAttach`), die das offizielle PlatformIO (noch) nicht bietet. Daher die **pioarduino**-Plattform (in `platformio.ini` gepinnt). Details: [`docs/DEPENDENCIES.md`](docs/DEPENDENCIES.md).

> **Hinweis zu `versions/` und `legacy/`:** Diese Ordner enthalten ältere Entwicklungsstände (v1–v4, UI-Tests). Der gebaute, lauffähige Stand ist `src/RadioGateway/`.

## Lizenz

MIT – siehe [LICENSE](LICENSE). © 2026 Yanis Ameseder

---

## Kontakt

**Entwickler:** Yanis Ameseder · **E-Mail:** [g4me.over.18@gmail.com](mailto:g4me.over.18@gmail.com)
