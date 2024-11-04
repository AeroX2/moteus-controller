This is a simple servo BLDC driver heavily based on the Moteus Controller https://github.com/mjbots/moteus/tree/main/hw/controller/r4.11

The main changes are
- Using a STSPIN32G4 instead of a microcontroller and DRV8353S
- Part changes and removal to allow for assembly by JLCPCB
- AS5048A accelerometer
- MAX3051 CAN driver
- Using SimpleFOC for the software
