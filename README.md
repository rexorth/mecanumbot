## Project Description
This project is a work in progress to build a pico-based robotic hardware platform (mostly) from scratch, including mechanical parts, component selection, PCB design, and system drivers.
There are several projects I am using heavily as references, including this [Arduino project](https://howtomechatronics.com/projects/arduino-mecanum-wheels-robot)
Once the robot can be controlled wirelessly, I intend to implement path planning, visual object recognition, and autonomous control.

## Current Capabilities
The robot is currently able to move and be controlled through a wired connection to an Xbox controller through the [Xinput driver](https://github.com/Ryzee119/tusb_xinput/tree/cfd83ba9b0809cf69f7b63d351f44ff73ebd0e30).

## TODO
- [ ] Implement wireless control using the [nRF24L01+ driver](https://nrf24.github.io/RF24/index.html)

- [ ] Bug: rotational speed of the robot is slow. Probably related to input normalization.
