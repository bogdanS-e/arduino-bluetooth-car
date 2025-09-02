# Arduino Bluetooth Car 🚗📡

This project is a simple Bluetooth-controlled car built on **Arduino**, using the **HC-06 Bluetooth module**.  
The car is controlled via the Android app **BT Car Controlled** (available on Google Play).  

---
## 🚀 Uploading the Sketch with PlatformIO

To build and upload your sketch to Arduino, run the following commands:

```bash
pio run
pio run --target upload
```

## 🔧 Hardware Requirements
- Arduino board (Uno / Nano / Mega etc.)
- HC-06 or HC-05 Bluetooth module
- Motor driver (e.g., L298N / L293D / similar)
- DC motors
- Power supply (batteries)
- Jumper wires

---

## 📲 Software Requirements
- Platform io
- Vs Code
- **BT Car Controlled** app from Play Market  
  👉 BT Car Controlled [Play Market](https://play.google.com/store/apps/details?id=com.giristuido.bluetooth.car.controller&hl=ru)

---

## 🔌 Wiring (Basic)

- HC-06 **TX → Arduino RX (D0 / D10 depending on SoftwareSerial)**
- HC-06 **RX → Arduino TX (D1 / D11 depending on SoftwareSerial)**
- HC-06 **VCC → 5V**
- HC-06 **GND → GND**
- Motor driver inputs → Arduino digital pins
- Motors → Motor driver outputs
- Battery → Motor driver VCC

---

## 📡 Bluetooth Data Format


- **Direction** (1 character) → movement type:
  - `F` = Forward
  - `B` = Backward
  - `L` = Left
  - `R` = Right
  - `S` = Stop

- **SpeedL** (2 digits) → Left motor speed (00–99)
- **SpeedR** (2 digits) → Right motor speed (00–99)

👉 Example commands:
- `F50R10` → Forward, left motor 50%, right motor 10%
- `B70L70` → Backward, both motors 70%
- `F00R00` → Stop

---

The **BT Car Controlled app** sends **6-character commands** to the Arduino via Bluetooth.  

