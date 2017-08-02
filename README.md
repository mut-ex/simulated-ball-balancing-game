# Simulated-Ball-Balancing-Game

The included device driver interfaces a MPU-6050 IMU module and a 8x8 LED Display (that is controlled using a MAX7219 IC) in order to implement a ball balance game.

At the start of each level, the level number is flashed on the display. After that the centre 4 LEDs are flashed to indicate the 'goal'. Then, the 'ball' is placed on a random location on the display.

The user is supposed to tilt the MPU-6050 in order to 'roll' the ball to the goal. If the ball stays there for 3 seconds, the level is won and the players moves on to the next level.

# Wiring
The wiring of the MPU-6050 module is as follows:

WARNING: Ensure that the IOREF jumper is on the 3.3V position.

VCC -- 5th pin of the Power header (female) on the Galileo board (3.3V)
GND -- 3rd pin of the Power header (female) on the Galileo board (GROUND)
SCL -- SCL pin on the Galileo board IO header.
SDA -- SDA pin on the Galileo board IO header.


The wiring of the 8x8 LED Display module is as follows:

VCC -- 5V external power supply
GND -- GROUND of external power supply. Remember to make the GROUNDS common through a jumper, i.e. the GROUND of the Galileo board and the GROUND of the external power supply.
DIN -- IO11 on the Galileo board
CS -- IO10 on the Galileo board
CLK -- IO13 on the Galileo board

# Usage
- Cross-compile the module and the user-space test program for the Galileo board using the accompanying Makefile. Transfer the files to the Galileo board.

- Install the imu module by typing and entering: insmod imu.ko

- To start the game, type and enter: ./test

To stop the game, press Ctrl + C.

To stop the kernel module, type: rmmod imu.ko

Note: If the kernel module is removed while the user program is running, the user program will automatically stop.
