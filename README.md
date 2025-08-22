
# DIY Marine Compass System

This project replaces a faulty fluxgate compass on a Robertson AP1000 Autohelm with a modern digital solution.

Author: Claude Sonnet & Paul Collister, SV Sister Midnight, paul@collisters.com
## Overview

- **Sensor:** [CMPS12 digital compass module](https://www.robot-electronics.co.uk/cmps12.html)
- **Controller:** Raspberry Pi Pico W microcontroller
- **Analog Output:** Two MCP4725 I2C DACs provide 5V-compatible sine and cosine signals for autopilot input (a dual DAC with reference input is recommended for future improvements).

## Features

- Accurate digital heading from the CMPS12
- Outputs analog sine and cosine signals via MCP4725 DACs
- WiFi connectivity for remote monitoring and NMEA 0183 data streaming
- Simple calibration and status feedback

## Hardware Connections

- **CMPS12**: Connected to I2C bus 0 (GPIO 0=SDA, GPIO 1=SCL)
- **MCP4725 DACs**: Connected to I2C bus 1 (GPIO 2=SDA, GPIO 3=SCL)
	- Address 0x60: Sine output
	- Address 0x61: Cosine output

## Software

- Written in MicroPython for the Pi Pico W
- Outputs NMEA HDT sentences over TCP (port 10110)
- Easy to configure WiFi credentials in `main.py`

## Resources

- https://www.robot-electronics.co.uk/files/cmps12.pdf
- [MCP4725 Datasheet (PDF)](https://cdn.sparkfun.com/datasheets/BreakoutBoards/MCP4725_2009.pdf)
- https://www.sparkfun.com/sparkfun-i2c-dac-breakout-mcp4725.html
- https://cdn.sparkfun.com/datasheets/BreakoutBoards/MCP4725_2009.pdf
- file:///Users/Paulc/Downloads/bst-bno055-ds000.pdf
- https://thepihut.com/products/raspberry-pi-pico-w?srsltid=AfmBOooGuRW-58-McknmdtbYEMGbDdnfv3R16uf0yszZ1yiBiJiKXnx8

## Getting Started

1. Flash MicroPython to your Pi Pico W.
2. Wire up the CMPS12 and MCP4725 modules as described above.
3. Edit `main.py` to set your WiFi credentials.
4. Upload the code to your Pico W.
5. Power up and monitor the serial output for status.

## Future Improvements

- Use a dual DAC with reference input for improved analog output
- Add web-based configuration and diagnostics


## AI and other issues.
This project was originally started without access to DACs, the MCP4922 is preferred. 
AI (Claude sonnet 4) wrote most of the code and the docs.
The docs refer to using PWM,  but the dacs used are on i2c. The basic Pi pico was chosen over the Pi zero in the end to improve on cost, and the pico is more suitable as a micronctroller with e2 prom
