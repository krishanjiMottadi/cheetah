# CHEETAH - 3D Printed RC Race Car (Arduino + DIY Remote)

A fully functional, high-speed 3D-printed RC race car controlled by a custom-built DIY remote using Arduino. Designed for makers, engineers, and RC enthusiasts, this project combines mechanical design, electronics, and wireless control into a sleek and powerful racing platform.

## 🏁 Project Overview

The **Cheetah** is a custom-designed 3D printed RC car chassis powered by brushless motors and an Arduino-compatible control system. Paired with a DIY handheld controller, it offers both performance and affordability, perfect for robotics and racing enthusiasts.

## 🚀 Features

- 🧩 **100% 3D Printed Chassis** – Custom, modular design for lightweight and durability  
- ⚙️ **High-Torque Steering System** – Robust steering powered by a standard servo  
- 🔋 **Brushless DC Motor with ESC** – For high-speed and smooth throttle control  
- 🎮 **DIY Remote Control** – Built using potentiometers, a joystick, and an Arduino Nano  
- 📡 **Wireless Communication** – Uses NRF24L01+ modules for low-latency control  
- 🔧 **Open-Source & Fully Customizable**

## 🔧 Components Used

### RC Car:
- Arduino Nano  
- Brushless Motor + 30A ESC  
- NRF24L01+ (Wireless Module)  
- Servo Motor (Steering)  
- LiPo Battery  
- 3D Printed Car Parts (STL files provided)

### Remote Controller:
- Arduino Nano  
- NRF24L01+  
- Joystick Module  
- Potentiometer (for throttle trim)  
- Push Buttons  
- OLED Display (optional)  
- 3D Printed Case

## 🛠️ 3D Printed Parts

The car and controller bodies are designed in CAD software and printed using PLA/ABS. STL files for:

- Car chassis, wheels, servo mounts  
- Controller case and knobs

> All STL files are organized under the `STL Files/` directory.


## 🎮 Control Logic

- Joystick controls the steering (X-axis)
- The throttle is controlled using a potentiometer or trigger
- Wireless data sent via NRF24L01+ to the RC car in real-time

## 🧠 How It Works

1. **DIY Controller** sends joystick data via NRF24L01+
2. **RC Car Arduino** receives commands and adjusts motor speed and steering accordingly
3. **ESC** controls the BLDC motor using PWM signals
4. **Servo** handles steering with proportional movement

## 🔋 Power Supply

- Car: 2S/3S LiPo battery (7.4V–11.1V)
- Controller: Rechargeable 9V or power bank (5V boost)

## 📦 Future Improvements

- Add telemetry (battery %, motor temp, speed)
- Upgrade to the FPV system for immersive control
- Integrate the mobile app using Bluetooth

## 🧑‍💻 Author

**M. Krishnaji**  
- GitHub: [krishanjiMottadi](https://github.com/krishanjiMottadi)  

## 📜 License

This project is licensed under the [MIT License](LICENSE).

---

> 🛠️ *This is an open hardware/software project. Contributions, forks, and modifications are highly welcome!*
