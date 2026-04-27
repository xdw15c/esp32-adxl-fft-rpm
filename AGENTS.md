# Project Guidelines

## Build And Validation

- Use `platformio run` for builds.
- Use `platformio run --target upload` only after checking the serial port in [platformio.ini](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\platformio.ini).
- Use `platformio device monitor --port <port> --baud 115200` for serial logs.
- There are no committed tests yet; validate behavior with a focused build and, when hardware is available, a short serial-monitor check.

## Source Of Truth

- Treat [src/main.cpp](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\src\main.cpp) and [platformio.ini](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\platformio.ini) as the current implementation source of truth.
- Treat [README.md](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\README.md) as design and hardware context. It describes planned ADXL345, OLED, and Modbus features that may not exist in the current firmware yet.
- Use [include/secrets.h.example](c:\Users\gcich\Documents\_QSYNC\Dokumenty\PlatformIO\Projects\esp32-c3-adxl\include\secrets.h.example) as the template for local credentials; do not replace it with real secrets.

## Firmware Conventions

- Keep feature toggles driven by `build_flags` macros such as `ENABLE_OLED` and `ENABLE_ADXL`; do not hardcode those settings in source files when a config flag already exists.
- Keep hardware defaults aligned with the current board setup: ESP32-C3, Arduino framework, I2C on GPIO8 and GPIO9, serial monitor at 115200.
- Preserve the current minimal runtime unless the task explicitly expands scope. The repo memory and current code indicate the implemented baseline is Wi-Fi plus a WebServer status page, not the full architecture described in the README.
- When adding functionality from the README plan, update code and configuration together so the documented feature flags, libraries, and endpoints remain consistent.

## Common Pitfalls

- `upload_port` and `monitor_port` are machine-specific. Verify them before upload instead of assuming `COM8` is valid.
- New work that depends on Wi-Fi credentials or HTTP auth should keep `include/secrets.h` local and out of version control.
- If you touch `platformio.ini`, preserve existing comments and grouped `build_flags`; they document which settings are optional modules versus board or timing configuration.