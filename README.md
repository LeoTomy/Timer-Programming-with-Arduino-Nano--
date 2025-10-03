# Task 3 – Timer Programming with Arduino Nano  

![License](https://img.shields.io/badge/license-MIT-green.svg)
![Top language](https://img.shields.io/github/languages/top/LeoTomy/Timer Programming with Arduino Nano)
![Platform](https://img.shields.io/badge/platform-Arduino%20Nano-orange.svg)  
![Status](https://img.shields.io/badge/status-Completed-success.svg)  

This project focuses on **Timer/Counter programming, PWM generation, and ADC integration** on the **Arduino Nano (ATmega328P)** without using Arduino libraries. All functionality is implemented at **register level**.  

---

## 📌 Overview  
The project demonstrates how to:  
- Read analog input values using the **ADC**.  
- Generate a **PWM signal** using timers to control a servo motor.  
- Use **voltage thresholds** to trigger LED behaviors (blinking or dimming).  
- Apply embedded C programming concepts directly on microcontroller registers.  

---

## ⚡ Features  
- **Potentiometer input (0–5 V)** → controls servo position (0°–180°).  
- **PWM output** drives a servo motor.  
- **LED behavior**:  
  - Below 2.5 V → LED dims proportionally.  
  - Above 2.5 V → LED blinks with 1s interval.  
- **No Arduino libraries used** → all code works at hardware register level.  

---

## 🛠️ Hardware Setup  
- **Microcontroller**: Arduino Nano (ATmega328P)  
- **Components**:  
  - Potentiometer (analog input)  
  - Servo motor (PWM output)  
  - LED + Resistor  
📷  ![Line Follower Robot](docs/robot.jpg)


---

## 🚀 How to Run  
1. Flash the C code to your Arduino Nano using **AVR-GCC** or Arduino IDE (register-level).  
2. Connect the hardware as described.  
3. Adjust the potentiometer and observe servo + LED behavior.  

---
