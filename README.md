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
