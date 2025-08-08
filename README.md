# Self-Balancing Robotic Bike

An Arduino-based embedded system that balances a robotic bike using real-time PID control.

## Features
- Real-time PID stabilization using MPU6050 sensor
- Modular embedded C code with PWM-based motor control
- UART serial communication with Python-based telemetry
- MATLAB-based modeling and PID gain tuning
- Upgrade-ready: interrupt-driven design, modular codebase

## Code Structure
- `main.ino`: Core control logic
- `telemetry_plot.py`: Real-time telemetry
- `PID_tuning.m`: MATLAB script for controller tuning

## Future Work 
- Kalman filter integration
- Remote control via RF24 or Bluetooth

## Requirements
1.)Arduino Uno/Nano

2.)MPU6050 Sensor

3.)Dual motor driver (L298N or similar)

4.)BO Motors (or equivalent)

5.)USB Cable for Serial

6.)Python + pyserial + matplotlib for telemetry

