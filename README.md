# heat-control
# Intelligent Heating Control System

An ESP32-based heating control system with temperature monitoring, state management, BLE communication, and visual feedback.

## 🚀 Features

- **State-based Temperature Control**: 6 operational states (Idle, Heating, Stabilizing, Target Reached, Overheat, Error)
- **Real-time Temperature Monitoring**: DS18B20 digital temperature sensor
- **BLE Communication**: Wireless status broadcasting and control
- **Visual Feedback**: RGB LED status indicators
- **Audio Alerts**: Buzzer for state change notifications
- **Safety Features**: Overheat protection and sensor fault detection
- **FreeRTOS Integration**: Multi-task architecture with timers
- **Serial Logging**: Detailed status and debugging information

## 🔧 Hardware Requirements

### Components
- ESP32 Development Board
- DS18B20 Temperature Sensor (waterproof version recommended)
- 5V Single Channel Relay Module
- RGB LED (Common Cathode)
- Piezo Buzzer
- 4.7kΩ Pull-up Resistor
- 220Ω Current Limiting Resistors (3x)
- Breadboard and Jumper Wires

### Wiring Diagram

```
ESP32          Component
-----          ---------
GPIO 4   ----> DS18B20 Data (with 4.7kΩ pullup to 3.3V)
GPIO 2   ----> Relay Control Pin
GPIO 5   ----> Red LED (via 220Ω resistor)
GPIO 18  ----> Green LED (via 220Ω resistor)
GPIO 19  ----> Blue LED (via 220Ω resistor)
GPIO 21  ----> Buzzer Positive

Power Connections:
3.3V     ----> DS18B20 VDD, LED Common Cathode
5V       ----> Relay VCC
GND      ----> All component grounds
```

## 📱 Software Requirements

### Arduino IDE Setup
1. Install ESP32 board package
2. Install required libraries:
   ```
   OneWire by Paul Stoffregen
   DallasTemperature by Miles Burton
   ESP32 BLE Arduino by Neil Kolban
   ```

### Platform.io (Alternative)
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    milesburton/DallasTemperature@^3.11.0
    paulstoffregen/OneWire@^2.3.7
    ESP32 BLE Arduino@^2.0.0
```

## 🖥️ Wokwi Simulation

### Quick Start
1. Open [Wokwi ESP32 Simulator](https://wokwi.com/projects/new/esp32)
2. Replace the default code with the provided heating control system code
3. Add the following components to your simulation:
   - ESP32 DevKit V1
   - DS18B20 Temperature Sensor
   - 5V Relay Module
   - RGB LED
   - Buzzer

### Simulation Link
```
(https://wokwi.com/projects/437083566515142657)
```


## 🏃‍♂️ Getting Started

### 1. Hardware Assembly
1. Connect components according to the wiring diagram
2. Double-check all connections
3. Ensure power supply can handle relay current requirements

### 2. Software Upload
1. Open the project in Arduino IDE
2. Select "ESP32 Dev Module" as the board
3. Choose the correct COM port
4. Upload the code to your ESP32
5. Open Serial Monitor (115200 baud) to view system logs

### 3. System Configuration
```cpp
// Temperature Thresholds (modify in code)
#define TARGET_TEMP         50.0f    // Target temperature in Celsius
#define TEMP_HYSTERESIS     2.0f     // Temperature hysteresis band
#define OVERHEAT_THRESHOLD  85.0f    // Safety cutoff temperature
#define RECOVERY_TEMP       75.0f    // Recovery temperature from overheat
```

## 📊 System States

| State | Description | LED Color | Heater Status |
|-------|-------------|-----------|---------------|
| **IDLE** | System ready, monitoring | Blue | OFF |
| **HEATING** | Actively heating to target | Purple (Red+Blue) | ON |
| **STABILIZING** | Fine-tuning near target | Blinking Yellow | ON/OFF |
| **TARGET_REACHED** | Maintaining target temp | Green | ON/OFF |
| **OVERHEAT** | Safety protection active | Blinking Red | OFF |
| **ERROR** | Sensor fault detected | Fast Blinking Red | OFF |

## 🔊 Audio Feedback

- **Heating Start**: Single 1000Hz beep
- **Target Reached**: Double 1500Hz beeps
- **Overheat Alert**: 5 rapid 2000Hz beeps

## 📡 BLE Communication

### Service UUID
```
Service: 12345678-1234-1234-1234-123456789abc
Characteristic: 87654321-4321-4321-4321-cba987654321
```

### Advertisement Data
The system broadcasts:
- Device Name: "HeatingController"
- Temperature reading
- Current state
- Heater status

### Mobile App Integration
You can develop a mobile app using:
- Flutter with `flutter_blue` plugin
- React Native with BLE libraries
- Native Android/iOS BLE APIs

## 🔒 Safety Features

### Overheat Protection
- Automatic heater shutdown at 85°C
- Visual and audio alerts
- Manual reset required for recovery

### Sensor Fault Detection
- Invalid reading detection
- Sensor disconnection monitoring
- Automatic failsafe mode activation

### Watchdog Monitoring
- System health checks every 10 seconds
- Task monitoring and recovery

## 📈 Performance Specifications

| Parameter | Specification |
|-----------|---------------|
| Temperature Accuracy | ±0.5°C |
| Response Time | <2 seconds |
| Operating Range | 0°C to 100°C |
| BLE Range | 10+ meters |
| Power Consumption | ~300mA @ 5V (heating) |
| Update Rate | 1 second |

## 🛠️ Development Guide

### Adding New Features

1. **Custom Temperature Profiles**
```cpp
struct TempProfile {
    float targetTemp;
    float hysteresis;
    unsigned long duration;
};
```

2. **WiFi Connectivity**
```cpp
#include <WiFi.h>
#include <WebServer.h>
// Add web server for remote control
```

3. **Data Logging**
```cpp
#include <SPIFFS.h>
// Log temperature data to flash memory
```

### Task Architecture
```
Core 0:
├── Temperature Reading Task (Priority 2)
├── State Manager Task (Priority 3)
└── Hardware Timers

Core 1:
├── BLE Communication Task (Priority 1)
├── Feedback Task (Priority 1)
└── Main Loop (Watchdog)
```

## 🐛 Troubleshooting

### Common Issues

**Temperature Reading Error**
```
Symptoms: "Invalid temperature reading" in serial
Solutions:
- Check DS18B20 wiring
- Verify 4.7kΩ pullup resistor
- Test sensor with multimeter
```

**BLE Connection Issues**
```
Symptoms: Device not discoverable
Solutions:
- Restart ESP32
- Clear Bluetooth cache on phone
- Check for interference
```

**Heater Not Responding**
```
Symptoms: Relay clicking but no heating
Solutions:
- Check relay wiring and power
- Verify heater element connectivity
- Test relay with multimeter
```

### Debug Commands
Enable debug mode by modifying:
```cpp
#define DEBUG_MODE 1
```

## 📊 Serial Output Example

```
=== Heating Control System Starting ===
Hardware initialized
BLE initialized and advertising
System Initialized Successfully
State change: UNKNOWN -> IDLE
[5] Temp: 23.4°C | State: IDLE (0s) | Heater: OFF | Target: 50.0°C
State change: IDLE -> HEATING
Heater ON
[15] Temp: 25.8°C | State: HEATING (10s) | Heater: ON | Target: 50.0°C
...
State change: HEATING -> TARGET_REACHED
Heater OFF
[180] Temp: 50.1°C | State: TARGET_REACHED (5s) | Heater: OFF | Target: 50.0°C
```

## 🚀 Future Enhancements

### Phase 1 (Immediate)
- [ ] Web interface for remote monitoring
- [ ] Temperature data logging to SD card
- [ ] Email notifications for alerts
- [ ] PID control algorithm implementation

### Phase 2 (Short-term)
- [ ] Multiple heating profiles
- [ ] Smartphone app development
- [ ] Cloud connectivity (AWS IoT/Firebase)
- [ ] Energy consumption monitoring

### Phase 3 (Long-term)
- [ ] Machine learning for predictive control
- [ ] Multi-zone heating support
- [ ] Smart home integration
- [ ] Industrial protocol support (Modbus, etc.)

## 📋 Testing Checklist

- [ ] Temperature sensor reading accuracy
- [ ] State transitions working correctly
- [ ] Heater control responding properly
- [ ] LED indicators showing correct states
- [ ] Buzzer alerts functioning
- [ ] BLE advertisement working
- [ ] Serial logging comprehensive
- [ ] Overheat protection activating
- [ ] Error recovery functioning
- [ ] System stability over 24 hours

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/new-feature`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/new-feature`)
5. Create Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👥 Support

For technical support and questions:
- Create an issue in the repository
- Contact: [your-email@example.com]
- Documentation: [link-to-docs]

## 🙏 Acknowledgments

- ESP32 community for excellent documentation
- Arduino ecosystem for simplifying embedded development
- FreeRTOS for robust task management
- Contributors and testers

---

**⚠️ Safety Warning**: This system controls heating elements. Always follow proper electrical safety procedures and local regulations. Never leave the system unattended during operation.
