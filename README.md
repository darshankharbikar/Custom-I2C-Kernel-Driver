# Objective
- Develop a minimal Linux kernel I2C driver for BME680 sensor on Raspberry Pi that exposes raw temperature ADC values via sysfs, with proper chip detection (ID 0x61), forced measurement triggering, and Device Tree compatibility ("bosch,bme680").	

## Bosch BME280 sensor
### Key Specifications
 Bosch BME680 is a 4-in-1 environmental sensor measuring
- temperature (-40 to +85°C), 
- humidity (0-100% RH), 
- pressure (300-1100 hPa), and
- indoor air quality (gas resistance 20kΩ-300MΩ).
- It features ultra-low power consumption (1µA for temperature-only forced mode) with I²C (up to 3.4MHz) and SPI interfaces, operating at 1.71-3.6V.​

### Operating Modes
- Sleep mode: No measurements, minimal power
- Forced mode: Single TPHG (Temperature/Pressure/Humidity/Gas) cycle, auto-return to sleep
- Parallel mode: Continuous TPHG measurements
- Chip ID: 0x61. Temperature data at registers 0x22-0x24 (MSB/LSB/XLSB), config via 0x72-0x75.

## Components required
- For BME680 Kernel Driver on Raspberry Pi:
- Bosch BME680 Sensor Module (breakout board with 0.1" headers)
- Raspberry Pi (any model with I2C: Pi 4B, Pi 3B+, etc.)
- Jumper Wires (4x female-to-female or male-to-female Dupont)
- 3.3V Power Supply (RPi 3.3V GPIO pin)

## Connections
BME680    →    Raspberry Pi GPIO
VCC       →    Pin 1 (3.3V)
GND       →    Pin 6 (GND)  
SDA       →    Pin 3 (GPIO 2, I2C1 SDA)
SCL       →    Pin 5 (GPIO 3, I2C1 SCL)

## Optional Components
- Pull-up Resistors (4.7kΩ on SDA/SCL) - usually built into BME680 breakout
- Breadboard for prototyping
- USB-Serial Adapter for debugging (optional)
- No level shifters needed - BME680 is 3.3V native.

## Connections
BME680 to Raspberry Pi Connections
I²C Interface (Recommended for Kernel Driver)
| BME680 Pin | Raspberry Pi Pin | GPIO  | Function  |
| ---------- | ---------------- | ----- | --------- |
| VCC        | Pin 1            | -     | 3.3V      |
| GND        | Pin 6 or 9       | -     | Ground    |
| SDA        | Pin 3            | GPIO2 | I²C Data  |
| SCL        | Pin 5            | GPIO3 | I²C Clock |

## Notes:
- Default I²C address: 0x77 (ADDR pin floating/high)
- Ground ADDR pin → 0x76
- No level shifters needed (native 3.3V)
- Pull-up resistors (4.7kΩ) usually on breakout board
- Verify: i2cdetect -y 1 shows 76/77
- Enable I²C on Raspberry Pi:
```sudo raspi-config  # Interface Options → I2C → Enable```
```Reboot```
, then:
```lsmod | grep i2c```

## Executions
1. Enable I²C on Raspberry Pi
```sudo raspi-config```
- Interface Options → I2C → Enable → Reboot

2. Verify Hardware Detection
``` i2cdetect -y 1    # Should show 76 or 77```

3. Compile Kernel Module
- Save code as bme680_driver.c
- compile where Makefile exist
```make``` 
- Creates bme680.ko

4. Load Driver
```sudo insmod bme680.ko```
```dmesg | tail -5    # "BME680 detected, chip ID: 0x61"```

5. Read Temperature
```cat /sys/bus/i2c/devices/1-0077/temp    # Raw ADC value```
- Example: 2100000 (~25°C uncalibrated)

6. Unload (Testing)
```sudo rmmod bme680_custom```

7. Device Tree Overlay (Permanent)
```
// /boot/overlays/bme680.dtbo
/dts-v1/;
/plugin/;
/ {
    compatible = "brcm,bcm2835";
    fragment@0 {
        target = <&i2c1>;
        __overlay__ {
            #address-cells = <1>;
            #size-cells = <0>;
            status = "okay";
            bme680@77 {
                compatible = "bosch,bme680";
                reg = <0x77>;
            };
        };
    };
};
```
```sudo reboot```
→ Driver auto-loads.
Expected Output:
```dmesg | grep bme680```
[  2.345] bme680_custom 1-0077: BME680 detected, chip ID: 0x61
```cat /sys/bus/i2c/devices/1-0077/temp``` 
2147483

- Save as bme680_driver.c, compile with ```make -C /lib/modules/$(uname -r)/build M=$(pwd)``` modules, load with ```sudo insmod bme680.ko```.

Issues and their debugging

1. No Device Detected (probe fails)
Symptom: ```dmesg``` shows nothing, no sysfs /temp file
Issues:
- I2C not enabled: ```i2cdetect -y 1``` shows no 76/77
- Wrong wiring: SDA/SCL swapped or loose connection
- Power: VCC not at 3.3V (measure with multimeter)
- Wrong address: Try grounding ADDR pin for 0x76

Debug:
- ```i2cdetect -y 1          # Must show 76 or 77```
- ```sudo i2cdump -y 1 0x77  # Raw register dump (0xD0=0x61 confirms)```

2. "Invalid chip ID" Error
Symptom: ```dmesg | grep "Invalid chip ID"```
- Fake/clone sensor (not genuine Bosch 0x61)
- Wrong sensor (BME280=0x60, BMP280=0x58)

Debug: i2cdump → check register 0xD0 value
3. Temp reads return 0 or -EIO
Symptom:```cat /sys/.../ temp → 0 or error```
- No measurement trigger (forced mode not set)
- Reading before 5ms delay completes
- Sensor stuck in sleep mode

Debug:
# Manual test in probe success:
```i2cset -y 1 0x77 0x74 0x24   # Trigger forced mode```
sleep 0.01
```i2cdump -y 1 0x77 0x22 0x24  # Check temp registers !=0```

4. Compile Errors
make: *** No rule to make target 'modules'
Fix:
# Install kernel headers
```sudo apt install raspberrypi-kernel-headers```
# Use correct path
```make -C /lib/modules/$(uname -r)/build M=$(pwd) modules```

5. Permission Denied on sysfs
Symptom: cat: /sys/.../temp: Permission denied
Fix: sudo chmod 644 /sys/bus/i2c/devices/1-0077/temp
6. Kernel Panic (rare)
Symptom: Oops on insmod
- Missing includes or kernel version mismatch
- Stack corruption in probe

Debug: ```dmesg | tail -20, check uname -r matches headers```

Quick Debug Checklist
1. i2cdetect -y 1              # → 76/77?
2. sudo insmod bme680.ko       # → no errors?
3. dmesg | tail -3             # → "BME680 detected"?
4. ls /sys/bus/i2c/devices/    # → 1-0077/temp?
5. cat /sys/bus/i2c/devices/1-0077/temp  # → ~2000000?

Most common fix order: Wiring → I2C enable → Headers → Address

Checklist

Hardware Setup ✓
 BME680 breakout board connected
 VCC → Pi Pin 1 (3.3V)
 GND → Pi Pin 6
 SDA → Pi Pin 3 (GPIO2)
 SCL → Pi Pin 5 (GPIO3)
 i2cdetect -y 1 shows 76 or 77
Software Prerequisites ✓
 I²C enabled: sudo raspi-config → Interface Options → I2C
 Kernel headers: sudo apt install raspberrypi-kernel-headers
 Reboot after I²C enable
Build & Load ✓
# Create directory, save bme680_driver.c
mkdir bme680_driver && cd bme680_driver
# Create Makefile:
echo 'obj-m += bme680_driver.o' > Makefile
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules

 ls shows bme680_driver.ko
 sudo insmod bme680_driver.ko → no errors
Driver Verification ✓
 ```dmesg | tail -3 → "BME680 detected, chip ID: 0x61"```
 ```ls /sys/bus/i2c/devices/ → 1-0076 or 1-0077```
 ```ls /sys/bus/i2c/devices/1-0077/``` → temp file exists

Functionality Test ✓
```cat /sys/bus/i2c/devices/1-0077/temp```
- Returns ~2000000 (raw ADC, not 0 or error)
- Multiple reads give consistent changing values
- Troubleshooting Quick Checks
❌ No 76/77? → Wiring/I2C enable
❌ "Invalid chip ID"? → Wrong sensor or fake
❌ Compile fail? → Missing headers  
❌ Temp=0? → Check forced mode trigger
❌ Permission denied? → `sudo chmod 644 /sys/.../temp`

Success Indicators
✓ i2cdetect:     76/77 detected
✓ dmesg:         "BME680 detected"  
✓ sysfs:         /sys/.../1-0077/temp exists
✓ cat temp:      21xxxxx (valid raw value)
✓ rmmod:         No errors

Artifacts

BME680 Kernel Driver Artifacts
1. Source Files
- bme680_driver.c      # Main kernel module source
- Makefile             # Build configuration
- bme680.dtbo          # Device Tree overlay (optional)

2. Compiled Outputs
- bme680_driver.ko     # Loadable kernel module
- bme680_driver.mod.c  # Module info
- bme680_driver.o      # Object file
- modules.order       # Build order
- Module.symvers       # Symbol versions

3. Makefile Content
-makefile
```
obj-m += bme680_driver.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean
```
4. Device Tree Overlay (bme680-overlay.dts)
```
/dts-v1/;
/plugin/;
/ {
    compatible = "brcm,bcm2835";
    fragment@0 {
        target = <&i2c1>;
        __overlay__ {
            #address-cells = <1>;
            #size-cells = <0>;
            status = "okay";
            bme680@77 {
                compatible = "bosch,bme680";
                reg = <0x77>;
            };
        };
    };
};
```
Compile:``` dtc -@ -I dts -O dtb -o bme680.dtbo bme680-overlay.dts```

5. Sysfs Artifacts (Runtime)
/sys/bus/i2c/devices/1-0077/
├── temp              # Raw temperature ADC
├── uevent
├── subsystem
└── power/

6. Log Outputs (Expected)
dmesg:
[  2.345678] bme680_custom 1-0077: BME680 detected, chip ID: 0x61

i2cdetect -y 1:
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
20:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- 77 -- -- -- -- -- -- -- -- 

7. Test Script (test_bme680.sh)
```
#!/bin/bash
echo "=== BME680 Kernel Driver Test ==="
i2cdetect -y 1 | grep 7
echo "Driver status: $(dmesg | tail -1 | grep BME680 || echo 'Not loaded')"
cat /sys/bus/i2c/devices/1-0077/temp 2>/dev/null || echo "Temp file missing"
```


