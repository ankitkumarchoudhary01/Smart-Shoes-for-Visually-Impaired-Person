# Smart Navigation Shoes - Setup Guide

## Hardware Assembly

### Step 1: Sensor Placement
1. **Front Ultrasonic**:  Toe area, facing forward
2. **Left Ultrasonic**: Left side, angled 30° outward
3. **Right Ultrasonic**: Right side, angled 30° outward
4. **IR Sensors**: Place close to ground for immediate obstacle detection

### Step 2: Vibration Motor Placement
1. **Front Motor**: Top of foot (instep area)
2. **Left Motor**: Left side of foot
3. **Right Motor**: Right side of foot

Ensure motors are in direct contact with skin/sock for better feedback. 

### Step 3: Electronics Assembly
1. Connect all sensors to ESP32 according to circuit diagram
2. Use transistors to drive vibration motors
3. Install battery and charging circuit
4. Test all connections with multimeter

### Step 4: Waterproofing
1. Use conformal coating on circuit boards
2. Seal all connections with heat shrink tubing
3. Place electronics in waterproof enclosure
4. Use IP67-rated connectors for external sensors

## Software Installation

### Step 1: Install Arduino IDE
```bash
# Download from https://www.arduino.cc/en/software
# Install ESP32 board support
```

### Step 2: Install Libraries
```cpp
// In Arduino IDE, go to Tools > Manage Libraries
// Install: 
// - ESP32 by Espressif Systems
// - BluetoothSerial (included with ESP32)
```

### Step 3: Upload Code
1. Open `smart_navigation_shoes.ino`
2. Select Board: "ESP32 Dev Module"
3. Select correct COM Port
4. Click Upload

### Step 4: Configure Settings
1.  Adjust distance thresholds in `config.h`
2. Calibrate sensors based on user testing
3. Customize vibration patterns

## User Testing Protocol

### Calibration Phase
1. Test obstacle detection accuracy
2. Adjust vibration intensity based on user feedback
3. Fine-tune distance thresholds
4. Test in various environments (indoor/outdoor)

### Validation Testing
1. Obstacle detection accuracy test (aim for >95%)
2. Response time measurement (<100ms)
3. Battery life test (target:  8+ hours)
4. User comfort and ergonomics assessment

## Troubleshooting

### Sensors Not Detecting
- Check wiring connections
- Verify sensor power supply (5V)
- Test sensors individually with Serial Monitor

### Weak Vibration
- Check transistor connections
- Verify motor voltage (3-3.3V)
- Increase PWM intensity in code

### Battery Draining Quickly
- Enable auto-sleep feature
- Reduce sensor polling frequency
- Check for short circuits

### Bluetooth Connection Issues
- Ensure Bluetooth is enabled on phone
- Check device name matches in code
- Restart both device and phone

## Maintenance

### Daily
- Charge battery when voltage drops below 3.5V
- Clean sensors with soft cloth

### Weekly
- Inspect all connections
- Test all vibration motors
- Check waterproof seals

### Monthly
- Recalibrate sensors
- Update firmware if available
- Replace worn components

## Safety Notes

⚠️ **Important:**
- This device is an assistive tool, not a replacement for white canes or guide dogs
- Always use in conjunction with other navigation aids
- Test thoroughly in safe environments before outdoor use
- Keep battery charged to ensure reliable operation
- Regular maintenance is critical for safety

## Support & Contribution

For issues, improvements, or questions:
- Report issues with detailed logs
- Share user feedback for improvements
- Contribute to code enhancements

**Developed with ❤️ for accessibility and independence**
