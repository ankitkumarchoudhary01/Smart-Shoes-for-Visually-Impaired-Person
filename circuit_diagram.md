## Smart Navigation Shoes - Circuit Connections

### ESP32 Pinout

#### Ultrasonic Sensors (HC-SR04)
**Front Sensor:**
- TRIG → GPIO 5
- ECHO → GPIO 18
- VCC → 5V
- GND → GND

**Left Sensor:**
- TRIG → GPIO 19
- ECHO → GPIO 21
- VCC → 5V
- GND → GND

**Right Sensor:**
- TRIG → GPIO 22
- ECHO → GPIO 23
- VCC → 5V
- GND → GND

#### IR Proximity Sensors
**Front IR:**
- OUT → GPIO 34
- VCC → 5V
- GND → GND

**Left IR:**
- OUT → GPIO 35
- VCC → 5V
- GND → GND

**Right IR:**
- OUT → GPIO 32
- VCC → 5V
- GND → GND

#### Vibration Motors (with transistor drivers)
**Front Motor:**
- Control → GPIO 25
- VCC → 3.3V (through transistor)
- GND → GND

**Left Motor:**
- Control → GPIO 26
- VCC → 3.3V (through transistor)
- GND → GND

**Right Motor:**
- Control → GPIO 27
- VCC → 3.3V (through transistor)
- GND → GND

**Note:** Use NPN transistors (2N2222 or similar) to drive motors

#### Buzzer
- Positive → GPIO 33 (through 100Ω resistor)
- Negative → GND

#### Power System
- Li-ion Battery (3.7V, 2000mAh+) → Buck-Boost Converter → 5V & 3.3V Rails
- Battery Monitor → Voltage Divider → GPIO 36 (ADC)
- Power LED → GPIO 2 (through 220Ω resistor)

### Components List
- 1x ESP32 Development Board
- 3x HC-SR04 Ultrasonic Sensors
- 3x IR Proximity Sensors
- 3x Vibration Motors (3V)
- 3x NPN Transistors (2N2222)
- 1x Active Buzzer (5V)
- 1x Li-ion Battery (3.7V, 2000mAh or higher)
- 1x Li-ion Charging Module (TP4056)
- 1x Buck-Boost Converter (3.7V to 5V)
- 1x 3.3V Voltage Regulator
- Resistors:  220Ω (2x), 100Ω (1x), 10kΩ (2x)
- Capacitors: 100µF (2x), 0.1µF (3x)
- Wires and connectors
- Waterproof enclosure
- Velcro straps or shoe mount