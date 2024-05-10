# Regulering ROV 2024 Project
This repository hosts the control software for the UiS Subsea ROV, slated for deployment in 2024. The software was developed as part of a bachelor's degree project.

## Getting Started
Key configurations, such as maximum power settings and enabling serial printing, can be adjusted in the `rovconfig.h` file.

## Pending Tasks
- **IMU Connection**: There is an existing issue where PWM behaves unexpectedly when SPI1 is enabled. This needs to be addressed to successfully connect the Inertial Measurement Unit (IMU).
- **Motor Soft Start**: Implement a soft start feature for the motors to prevent abrupt movements at startup.