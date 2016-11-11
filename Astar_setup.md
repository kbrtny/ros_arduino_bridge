# ROS_Arduino_bridge: A-Star I2C

This ros_arduino_bridge repo uses Pololu libraries to provide the I2C
connection between their A-Star 32U4 robot controller board mounted on a Raspberry
Pi computer.  I have successfully implemented it on a Pi 3 running ROS Kinetic.

## Setting up the Arduino IDE

The I2C implementation is based on Pololu's
[blog](https://www.pololu.com/blog/577/building-a-raspberry-pi-robot-with-the-a-star-32u4-robot-controller#)
post on building a robot with the A-Star board mounted on a Raspberry Pi.  Therein they provide references to the [A-Star board](https://www.pololu.com/docs/0J66) and how to setup the [Arduino Library](https://www.pololu.com/docs/0J66/5) for the Arduino IDE.

Having done this, download (clone) this repo (to a directory `~/Git/`, say) and open the contents of `ros_arduino_bridge/ros_arduino_firmware/src/libraries/ROSArduinoBridge` in your Arduino IDE.  You will compile and load `ROSArduinoBridge.ino` onto the A-Star board.  The A-Star buzzer will sound a little chime when it successfully boots up.  The `pI2C.h` (for Pololu I2C, or Pi I2C) contains the I2C data struct and communication for the A-Star.

## Setting up the Raspberry Pi

The Pololu [blog](https://www.pololu.com/blog/577/building-a-raspberry-pi-robot-with-the-a-star-32u4-robot-controller#)
also covers how to set up the Pi's I2C driver with these lines in the Pi's `/boot/config` file (the Pi's version of a BIOS):

```
dtparam=i2c_arm=on                # uncomment this
dtparam=i2c_arm_baudrate=400000   # add this
```
Upon reboot of the Pi, I find that the I2C driver fails to load, so I have to load it manually:
```
sudo modprobe --first-time i2c-dev
```
after which you can see if it is running by listing I2C devices:
```
i2cdetect -l
```
and, presuming your I2C bus is 1, you can probe it with:
```
i2cdetect 1
```

### Testing the Pi's connection to the A-Star

In the directory `ros_arduino_bridge/ros_arduino_python/src/ros_arduino_python/` are the pertinent files that control the I2C communication with the A-Star.  Particularly `a_star.py` and `arduino_a_starbus.py`.  While experimenting, you can run the `arduino_a_starbus.py` script on the Pi as a stand-alone program to test the I2C communication:
```
python arduino_a_starbus.py
```
This will connect to the A-Star, run the motors, report encoder counts, battery level, and analog inputs (i.e. raw IR sensor input), blink the LEDs and shut-down.  Since it is just python (no ROS), this is an easy file to edit and experiment with controlling the A-Star from the Pi.

### Setting up the ROS node on the Pi

Create a link to this repo in your `~/catkin_ws/src/` directory on your Pi and build it with:
```
cd ~/catkin_ws/src
ln -s ~/Git/ros_arduino_bridge .  # create link
cd ..
. devel/setup.bash
catkin_make
```
You should then be able to launch roscore to communicate with the A-Star.  I have adapted Mark Rose's [prsg2_robot](https://github.com/jdsalmonson/prsg2_robot) launch setup to do this.
