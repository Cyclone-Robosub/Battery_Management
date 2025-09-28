## Introduction
Based on the circuit design of the 2024-2025 season (communication with an Arduino Nano), this battery management solution would rely upon an ADC reading of the voltage(initial values) and the current (Coulomb counting). This solution has the ability to send notifications over ROS of the battery's current charge and danger zones of the battery.
The battery solution assumes the battery is always discharging and will never have the ability to charge the battery up during a run.
