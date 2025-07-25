# MQ-135 Gas Sensor Interface with NodeMCU ESP8266

This project provides an enhanced implementation for interfacing the MQ-135 gas sensor with a NodeMCU ESP8266. It includes preheat handling, EEPROM-based calibration, filtered sensor readings, and real-time gas concentration reporting with warning and danger thresholds.

## Features

- EEPROM storage for calibrated R₀
- Rs/R₀ calculation and live sensor reading
- Outlier filtering to improve accuracy
- Serial commands for calibration, reset, and info
- Threshold-based alerts for multiple gases
- Preheat handling before valid reading begins

## Supported Gases

| Gas      | Unit | Warning | Danger |
|----------|------|---------|--------|
| NH₃      | ppm  | 25.0    | 50.0   |
| NOx      | ppm  | 5.0     | 10.0   |
| CO₂      | ppm  | 1000.0  | 5000.0 |
| Alcohol  | ppm  | 100.0   | 500.0  |
| Benzene  | ppm  | 1.0     | 5.0    |

## Hardware Requirements

- NodeMCU ESP8266 or any Arduino-compatible board with analog input
- MQ-135 Gas Sensor
- Jumper wires
- Breadboard or sensor mount

## Wiring

| MQ-135 Pin | NodeMCU Pin |
|------------|-------------|
| VCC        | 3.3V        |
| GND        | GND         |
| AOUT       | A0          |

## How to Use

1. Upload the code using the Arduino IDE (baud rate: 115200)
2. Monitor the serial output to observe gas levels
3. Type commands in the serial monitor:
   - `calibrate` — calibrates R₀ in clean air
   - `reset` — clears saved calibration
   - `info` — shows current status

## Notes

- Ensure the sensor is in clean air before calibration
- Preheat takes ~20 seconds after boot
- CO₂ percentage is estimated (ppm to % conversion)

## License

This project is licensed for **personal and educational use only**. Commercial or public redistribution is prohibited.
