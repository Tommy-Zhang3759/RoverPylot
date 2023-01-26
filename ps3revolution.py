#!/usr/bin/env python

'''
ps3revolution.py Drive the Brookstone Rover Revolution via the P3 Controllerj
Copyright (C) 2014 Simon D. Levy
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
'''

# You may want to adjust these for your own controller
BUTTON_STEALTH = 1  # Circle button toggles stealth mode
BUTTON_TURRET = 3  # Square button toggles turret camera
AXIS_PAN_HORZ = 0  # Left joystick controls turret pan
AXIS_PAN_VERT = 1  # and tilt

# Avoid button bounce by enforcing lag between button events
MIN_BUTTON_LAG_SEC = 0.5

# Avoid close-to-zero values on axis
MIN_AXIS_ABSVAL = 0.01

# Threshold for faster speed
SPEED_THRESH = 0.5

# For FFPLAY
FRAMERATE = 16
DELAY_SEC = 4

from rover import Revolution

import time
import keyboard
import sys
import signal
import subprocess
import tempfile
import os


# Supports CTRL-C to override threads
def _signal_handler(signal, frame):
    frame.f_locals['rover'].close()
    sys.exit(0)


# Rover subclass for PS3 + OpenCV
class PS3Rover(Revolution):

    def __init__(self, tmpfile):

        # Set up basics
        Revolution.__init__(self)
        self.wname = 'Rover Revolution'

        # Set up controller using PyGame
        '''pygame.display.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()'''

        # Defaults on startup: no stealth; driving camera
        self.stealthIsOn = False
        self.usingTurret = False

        # Tracks button-press times for debouncing
        self.lastButtonTime = 0

        # Saves data to video file for display
        self.tmpfile = tmpfile

        #init keyboard status array
        i = 0
        for i in range(0xff):
            self.keyboard_status.append(False)

    keyboard_status = []

    def keyExpress(self, keyEvent):
        speed = False
        changed = False
        if(keyEvent.event_type == "up"):
            if self.keyboard_status[keyEvent.scan_code] != False:
                self.keyboard_status[keyEvent.scan_code] = False
                changed = True

        elif(keyEvent.event_type == "down"):
            if self.keyboard_status[keyEvent.scan_code] != True:
                self.keyboard_status[keyEvent.scan_code] = True
                changed = True
        
        if self.keyboard_status[51] == True:
            self.drive(0,0,speed)
            return "stop"

        if changed:
            print("KEYBOARD EVENT:\nscancode:",keyEvent.scan_code,"\nevent type:",keyEvent.event_type)
            if(self.keyboard_status[126] == True and self.keyboard_status[123] == True):
                self.drive(1,-1,speed)
                print ("go forward and turn left")
            elif(self.keyboard_status[126] == True and self.keyboard_status[124] == True):
                self.drive(1,1,speed)
                print ("go forward and turn right")
            elif(self.keyboard_status[125] == True and self.keyboard_status[123] == True):
                self.drive(-1,-1,speed)
                print ("go backward and turn left")
            elif(self.keyboard_status[125] == True and self.keyboard_status[124] == True):
                self.drive(-1,1,speed)
                print ("go backward and turn right")

            elif(self.keyboard_status[126] == True):
                self.drive(1,0,speed)
                print ("go forward")
            elif(self.keyboard_status[125] == True):
                self.drive(-1,0,speed)
                print ("go backward")
            elif(self.keyboard_status[123] == True):
                self.drive(0,-1,speed)
                print ("turn left")
            elif(self.keyboard_status[124] == True):
                self.drive(0,1,speed)
                print ("turn right")
            else:
                self.drive(0,0,speed)
                print ("immovability")
 
    # Automagically called by Rover class
    def processVideo(self, h264bytes, timestamp_msec):

        # Update controller events
        '''pygame.event.pump()'''

        # Toggle stealth mode (lights off / infrared camera on)
        '''self.stealthIsOn = self.checkButton(self.stealthIsOn, BUTTON_STEALTH, self.turnStealthOn, self.turnStealthOff)
        '''

        # Toggle stealth mode (lights off / infrared camera on)
        '''self.usingTurret = self.checkButton(self.usingTurret, BUTTON_TURRET, self.useTurretCamera,
                                            self.useDrivingCamera)
        '''

        
        # Use right joystick to drive
        axis2 = self.get_axis(2)
        axis3 = self.get_axis(3)
        goslow = False if abs(axis3) > SPEED_THRESH or abs(axis2) > SPEED_THRESH else True
        wheeldir = -self.axis_to_dir(axis3)
        steerdir = self.axis_to_dir(axis2)
        #self.drive(wheeldir, steerdir, goslow)

        # time.sleep(1)

        #self.drive(1,-1,True)

        keyboard.hook(self.keyExpress)

        # if keyboard.is_pressed('E+S+C'):
        #     print('ESC')

        # time.sleep(1)

        # self.drive(-1,0,True)

        # Use left joystick to control turret camera
        axis0 = self.axis_to_dir(self.get_axis(AXIS_PAN_HORZ))
        #self.moveCameraHorizontal(-axis0)
        axis1 = self.axis_to_dir(self.get_axis(AXIS_PAN_VERT))
        #self.moveCameraVertical(-axis1)
        #print(time.time())
        # Send video through pipe
        #print(self.tmpfile.name)
        self.tmpfile.write(h264bytes)
        return

    # Converts axis value to direction
    def axis_to_dir(self, axis):

        d = 0

        if axis > 0:
            d = 1
        elif axis < 0:
            d = -1

        return d

    # Returns axis value when outside noise threshold, else 0
    def get_axis(self, index):

        #value = self.controller.get_axis(index)
        value = 1

        if value > MIN_AXIS_ABSVAL:
            return value
        elif value < -MIN_AXIS_ABSVAL:
            return value
        else:
            return 0

    # Handles button bounce by waiting a specified time between button presses
    '''def checkButton(self, flag, buttonID, onRoutine=None, offRoutine=None):
        if self.controller.get_button(buttonID):
            if (time.time() - self.lastButtonTime) > MIN_BUTTON_LAG_SEC:
                self.lastButtonTime = time.time()
                if flag:
                    if offRoutine:
                        offRoutine()
                    flag = False
                else:
                    if onRoutine:
                        onRoutine()
                    flag = True
        return flag
    '''

# main -----------------------------------------------------------------------------------

if __name__ == '__main__':
    # Create a named temporary file for video stream
    tmpfile = tempfile.NamedTemporaryFile()

    # Create a PS3 Rover object
    rover = PS3Rover(tmpfile)

    # Set up signal handler for CTRL-C
    signal.signal(signal.SIGINT, _signal_handler)

    # Wait a few seconds, then being playing the tmp video file
    time.sleep(DELAY_SEC)
    cmd = 'ffplay -window_title Rover_Revolution -framerate %d %s' % (FRAMERATE, tmpfile.name)
    FNULL = open(os.devnull, 'w')
    subprocess.call(cmd.split(), stdout=FNULL, stderr=subprocess.STDOUT)

    # Shut down Rover
    rover.close()
