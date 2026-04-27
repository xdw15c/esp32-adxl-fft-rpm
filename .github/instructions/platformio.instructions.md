---
description: "Use when editing platformio.ini, changing PlatformIO environments, adjusting build_flags, serial ports, board settings, or ESP32-C3 firmware build configuration."
name: "PlatformIO Firmware Config"
applyTo: "platformio.ini"
---

# PlatformIO Configuration Guidelines

- Keep the environment aligned with the current hardware unless the task explicitly changes boards: ESP32-C3, Arduino framework, and the libraries declared in [platformio.ini](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\platformio.ini).
- Treat `upload_port` and `monitor_port` as developer-machine settings. Update them only when the task is explicitly about local hardware setup, and avoid assuming `COM8` is portable.
- Prefer changing behavior through `build_flags` macros before adding hardcoded constants in [src/main.cpp](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\src\main.cpp).
- Preserve the existing grouped comment structure in `build_flags`; it is the quick map of optional modules, I2C pin configuration, and timing values.
- Before adding new libraries or config flags, check whether the planned feature is already described in [README.md](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\README.md) and whether the current code in [src/main.cpp](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\src\main.cpp) actually implements it.