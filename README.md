# ESP32 Smart Fan Controller

*A sophisticated PWM fan controller with environmental monitoring and beautiful real-time visualization*

![Build Status](https://img.shields.io/badge/build-passing-brightgreen) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.0-blue) ![Python](https://img.shields.io/badge/python-3.8+-blue) ![GTK4](https://img.shields.io/badge/GTK-4.0-orange)

The **ultimate desktop fan controller** combining precision hardware control, beautiful software visualization, and environmental monitoring into one elegant system. Perfect for anyone who wants their cooling solution to be both functional and visually stunning.

## Features

### ESP32 Firmware
- **Precise PWM Control**: 0-100% variable speed using LEDC at 25kHz
- **Real-time RPM Monitoring**: Pulse counter with tachometer feedback
- **Smart Button Interface**: Cycles through 5 speed levels (0%, 25%, 50%, 75%, 100%)
- **Visual LED Feedback**: Blinks 1-5 times to indicate current speed level
- **Environmental Sensing**: SHT31-D temperature & humidity monitoring with CRC validation
- **Persistent Memory**: NVS storage remembers last speed across power cycles
- **Robust Networking**: WiFi auto-reconnect + MQTT with JSON messaging
- **Network Discovery**: mDNS available as `esp32-fan-controller.local`

### Beautiful GTK4 GUI
- **Modern Libadwaita Design**: Native GTK4/Adwaita styling with proper theming
- **Elegant Loading**: Centered spinner with smooth transition to content
- **Live Dashboard**: Real-time speed, RPM, temperature (°F), and humidity display
- **Smooth Animations**: All values count up/down smoothly over 2.5 seconds
- **Activity Indicator**: Subtle dot blinks when new data arrives
- **Smart Controls**: Full-width slider with conflict-free user/MQTT updates
- **Historical Visualization**: 
  - **Real-time Graph**: 10-minute rolling history of all 4 metrics
  - **Current Value Dots**: Color-coded dots with metric labels at graph endpoints
  - **Theme Integration**: Beautiful card-style graph container
  - **Live Updates**: Graph updates every 5 seconds with smooth rendering

## Hardware Configuration

### Components Required
- **ESP32 Development Board** (any variant with sufficient GPIO)
- **4-pin PC Fan** (12V power, 3.3V logic-compatible PWM/tach)
- **SHT31-D Temperature/Humidity Sensor**
- **Push Button** (momentary, normally open)
- **LED** (for status indication)
- **Resistors**: 10kΩ (pull-up), 220Ω (LED current limiting)

### Pin Configuration
| Component | GPIO Pin | Description |
|-----------|----------|-------------|
| PWM Output | GPIO 6 | 25kHz PWM signal for fan speed control |
| Tachometer | GPIO 7 | RPM measurement input (pulse counter) |
| Control Button | GPIO 16 | Speed preset cycling button |
| Status LED | GPIO 15 | Visual feedback for speed changes |
| SHT31-D SDA | GPIO 9 | I2C data line |
| SHT31-D SCL | GPIO 8 | I2C clock line |

### Wiring Diagram
```
ESP32                    4-Pin PC Fan
------                   ------------
GPIO 6  ──────────────── PWM (Pin 4)
GPIO 7  ──────────────── TACH (Pin 3)
3.3V    ──────────────── VCC (Pin 2)
GND     ──────────────── GND (Pin 1)

ESP32                    SHT31-D
------                   -------
GPIO 9  ──────────────── SDA
GPIO 8  ──────────────── SCL
3.3V    ──────────────── VDD
GND     ──────────────── GND

ESP32                    Button & LED
------                   -----------
GPIO 16 ──[10kΩ]─── 3.3V    Button
GPIO 16 ──────────────── Button (other terminal to GND)
GPIO 15 ──[220Ω]─── LED ─── GND
```

## Quick Start

### Prerequisites
- **ESP-IDF v5.0+** installed and configured
- **Python 3.8+** with pip
- **GTK4 development libraries** (see installation section)

### 1. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 2. Configure WiFi and MQTT
Edit the WiFi and MQTT settings in `main/main.c`:
```c
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASS "YourWiFiPassword"
#define MQTT_BROKER "broker.hivemq.com"  // or your MQTT broker IP
#define MQTT_PORT 1883
```

### 3. Build and Flash ESP32 Firmware
```bash
cd /path/to/quietsmart
idf.py build
idf.py flash monitor
```

### 4. Launch the GUI
```bash
python3 fan_control_gtk4.py
```

### 5. Desktop Integration (Optional)
```bash
# Copy desktop file for system integration
cp fan-controller.desktop ~/.local/share/applications/
```

## Installation

### Ubuntu/Debian
```bash
# Install GTK4 development libraries
sudo apt update
sudo apt install libgtk-4-dev libadwaita-1-dev python3-gi python3-gi-cairo gir1.2-gtk-4.0 gir1.2-adw-1

# Install ESP-IDF
curl -fsSL https://dl.espressif.com/dl/esp-idf/releases/esp-idf-v5.0.4.tar.gz | tar -xz
cd esp-idf-v5.0.4
./install.sh esp32
source export.sh
```

### Arch Linux
```bash
# Install GTK4 and Python bindings
sudo pacman -S gtk4 libadwaita python-gobject python-cairo

# Install ESP-IDF via AUR
yay -S esp-idf
source /opt/esp-idf/export.sh
```

### macOS
```bash
# Install GTK4 via Homebrew
brew install gtk4 libadwaita pygobject3

# Install ESP-IDF
brew install espressif/esp-idf/esp-idf
source $HOMEBREW_PREFIX/share/esp-idf/export.sh
```

## MQTT Protocol

### Status Updates (Published every 5 seconds)
**Topic**: `fan/status`
```json
{
  "speed": 75,
  "rpm": 2450,
  "temp_c": 23.5,
  "humidity": 45.2,
  "timestamp": 1234567890
}
```

### Speed Control (Subscribe for commands)
**Topic**: `fan/control`
```json
{"speed": 75}
```

### Example MQTT Commands
```bash
# Set fan to 50% speed
mosquitto_pub -h broker.hivemq.com -t fan/control -m '{"speed": 50}'

# Monitor status updates
mosquitto_sub -h broker.hivemq.com -t fan/status
```

## Control Methods

1. **Precision Slider**: Continuous 0-100% control via beautiful GUI
2. **Physical Button**: Quick preset cycling (0%, 25%, 50%, 75%, 100%) with LED confirmation
3. **MQTT Integration**: JSON API for automation and remote control
4. **Network Discovery**: Access via `esp32-fan-controller.local`

## Project Structure

```
quietsmart/
├── main/
│   ├── main.c                 # ESP32 firmware with sensor integration
│   ├── CMakeLists.txt         # Build configuration with I2C support
│   └── idf_component.yml      # Component dependencies
├── fan_control_gtk4.py        # Primary GUI with graphs and animations
├── fan_control.py             # Backup tkinter GUI
├── fan-controller.desktop     # Linux desktop launcher
├── requirements.txt           # Python dependencies
├── CMakeLists.txt             # Main project build file
└── README.md                  # This file
```

## What Makes This Special

- **Real-time Visualization**: See your fan's performance history at a glance
- **Environmental Awareness**: Monitor temperature and humidity trends  
- **Beautiful Interface**: Modern GTK4 design that feels like a native app
- **Data-Driven**: 10 minutes of rolling history with smooth animations
- **Smart Synchronization**: All animations sync to MQTT update intervals
- **Persistent Settings**: Remembers your preferences across power cycles
- **Visual Feedback**: LED patterns, activity indicators, and smooth transitions

## Development

### Building the Firmware
```bash
# Clean build
idf.py fullclean build

# Flash and monitor
idf.py flash monitor

# Configuration menu
idf.py menuconfig
```

### GUI Development
```bash
# Run with debug output
python3 fan_control_gtk4.py --debug

# Test MQTT connectivity
python3 -c "import paho.mqtt.client as mqtt; print('MQTT OK')"
```

### Customization
- **PWM Frequency**: Modify `LEDC_FREQUENCY` in `main/main.c`
- **Update Intervals**: Adjust timing constants in both firmware and GUI
- **Theme Colors**: Customize graph colors in `fan_control_gtk4.py`
- **MQTT Topics**: Change topic names in both firmware and GUI code

## Troubleshooting

### Common Issues

**ESP32 won't connect to WiFi**
- Check SSID/password in `main/main.c`
- Ensure 2.4GHz network (ESP32 doesn't support 5GHz)
- Monitor serial output for error messages

**GUI shows "Connecting..."**
- Verify MQTT broker is accessible
- Check firewall settings
- Ensure ESP32 is publishing to correct topic

**Fan not spinning**
- Verify 12V power supply for fan
- Check PWM wiring (GPIO 6)
- Ensure fan is 4-pin PWM compatible

**No RPM readings**
- Check tachometer wiring (GPIO 7)
- Verify fan has tach output (4-pin fans)
- Monitor serial output for pulse counts

## License

This project is open source. Feel free to modify and distribute according to your needs.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Support

- **Hardware Issues**: Check wiring and power connections
- **Software Issues**: Review serial monitor output
- **GUI Issues**: Ensure GTK4 and dependencies are properly installed

---

*Built with ESP-IDF, GTK4, Libadwaita, Cairo Graphics, and lots of attention to detail!*
