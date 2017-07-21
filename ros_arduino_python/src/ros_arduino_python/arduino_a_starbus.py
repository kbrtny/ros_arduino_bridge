#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import thread
from math import pi as PI, degrees, radians
import ctypes
import os
import time
import sys, traceback
#from smbus import SMBus
from a_star import AStar
from arduino_driver import Arduino

# TODO:  implement cmd in I2C.h

class ArduinoAStarBus(Arduino):
    def __init__(self, port = 1, device = 0x42):
        self.retry_count = 3
        self.port = port
        self.device = device
        self.a_star = None
        self.base_init()

    def connect(self):
        if self.a_star != None:
            self.a_star.close()

        self.a_star = AStar(self.port) #SMBus(self.port)

    def calculate_fletcher16(self, buff):
        s1 = 0
        s2 = 0

        for b in buff:
            s1 += b
            s2 += s1

        ret = (s2 & 0xff) << 8 | (s1 & 0xff)
        # print "calculate_fletcher16 retuns %d" % ret

        return ret


    def handle_exeception(self, e):
        if e.__class__.__name__ == "IOError":
            # print "handle_exeception?!"
            try:
                self.a_star.close()
            except Exception as e1:
                print "handle_exeception - close execption " + e1.__class__.__name__
                pass
            try:
                self.a_star = AStar(self.port)
            except Exception as e2:
                print "handle_exeception - open execption " + e2.__class__.__name__
                pass

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        # print "Updating PID parameters"
        self.mutex.acquire()

        self.a_star.update_pid(Kp, Kd, Ki, Ko)

        self.mutex.release()
        return True

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return 0;

    def get_encoder_counts(self):
        self.mutex.acquire()

        l_value, r_value = self.a_star.get_encoder_counts()

        self.mutex.release()

        return [ l_value, r_value ]

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        self.mutex.acquire()

        self.a_star.reset_encoders()

        self.mutex.release()
        return True

    def drive(self, left, right):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        # print "drive %d:%d" % (left, right)

        self.mutex.acquire()
        self.a_star.motors(left, right)
        self.mutex.release()

        return True

    def analog_read(self, pin):
        #self.mutex.acquire()
        if pin == 106:
            return self.a_star.read_battery_millivolts()
        else:
            return self.a_star.read_analog()[pin]

    def  analog_write(self, pin, value):
        return True

    def digital_read(self, pin):
        # A-star button pins a,b,c = 100,101,102:
        if pin >= 100 and pin <= 102:
            return self.a_star.read_buttons()[pin-100]
        return 0

    def digital_write(self, pin, value):
        if pin >= 103 and pin <= 105:
            leds = 3*[False]
            leds[pin-103] = bool(value)
            self.mutex.acquire()
            self.a_star.leds(*leds)
            self.mutex.release()
        return True

    def pin_mode(self, pin, mode):
        return True

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position in degrees (integer between 0 & 180)
        '''
        if id == 0:
            self.mutex.acquire()
            self.a_star.write_servo_left(pos)
            self.mutex.release()
        elif id == 1:
            self.mutex.acquire()
            self.a_star.write_servo_right(pos)
            self.mutex.release()
        return True

    def servo_read(self, id):
        ''' Usage: servo_read(id)
            Position in degrees (integer between 0 & 180)
        '''
        if id >= 0 and id < 2:
            return self.a_star.read_servos()[id]
        return 0

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return 0

    def get_maxez1(self, triggerPin, outputPin):
        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
            sensor connected to the General Purpose I/O lines, triggerPin, and
            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
            power up, otherwise it wont range correctly for object less than 6
            inches away! The sensor reading defaults to use English units
            (inches). The sensor distance resolution is integer based. Also, the
            maxsonar trigger pin is RX, and the echo pin is PW.
        '''
        return 0


""" Basic test for connectivity """
if __name__ == "__main__":
    myArduino = ArduinoAStarBus(port = 1)
    myArduino.connect()

    print "Sleeping for 1 second..."
    time.sleep(1)

    print "Drive motor 1..."
    myArduino.a_star.motors(100,0)
    time.sleep(2.0)
    print "Drive motor 2..."
    myArduino.a_star.motors(0,100)
    time.sleep(2.0)
    myArduino.a_star.motors(0,0)

    print "Encoder counts:",myArduino.a_star.get_encoder_counts()

    print "Battery in millivolts:",myArduino.a_star.read_battery_millivolts()

    for i in range(5):
	raw_input("Press enter to read analog pins...")
        print i,"Reading on analog ports 0,1,2:", myArduino.analog_read(0), \
                                            myArduino.analog_read(1), \
                                            myArduino.analog_read(2)
    #print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        #myArduino.digital_write(13, 1)
        myArduino.a_star.leds(0,0,0)
        time.sleep(1.0)
        myArduino.a_star.leds(1,1,1)
        time.sleep(1.0)
    myArduino.a_star.leds(0,0,0)
    #print "Current encoder counts", myArduino.encoders()

    print "Move servoes:"
    left_srv, rght_srv = myArduino.a_star.read_servos()
    print "Servos: left = ", left_srv, " right = ", rght_srv
    myArduino.a_star.write_servos(120, 60);
    time.sleep(1.0);

    left_srv, rght_srv = myArduino.a_star.read_servos()
    print "Servos: left = ", left_srv, " right = ", rght_srv

    myArduino.a_star.write_servos(150, 30);
    time.sleep(1.0);

    left_srv, rght_srv = myArduino.a_star.read_servos()
    print "Servos: left = ", left_srv, " right = ", rght_srv

    print "Connection test successful.",

    print "Shutting down Arduino."
