#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

#GPIO 23 and 24 are battery cells
#GPIO 8 battery charging
#GPIO 25 battery charged
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN)
GPIO.setup(24, GPIO.IN)
GPIO.setup(25, GPIO.IN)
GPIO.setup(8,  GPIO.IN) 

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def battery_reader():
    rospy.init_node('battery_reader', anonymous=True)
    rate = rospy.Rate(0.5)
    print("Starting battery_reader node by Luis Nava...")

    try:
        while not rospy.is_shutdown():
            if GPIO.input(23) or GPIO.input(24):
                print(bcolors.BOLD + bcolors.FAIL + "BATTERY CRITICALLY LOW :O" + bcolors.ENDC)
            if GPIO.input(25) and GPIO.input(8):
                pass
            else:
                if GPIO.input(25):
                    print(bcolors.BOLD + bcolors.OKGREEN + "BATTERY CHARGED COMPLETED :)" + bcolors.ENDC)
                if GPIO.input(8):
                    print(bcolors.WARNING + "BATTERY IS CHARGING" + bcolors.ENDC)
            rate.sleep()

    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        battery_reader()
    except rospy.ROSInterruptException:
        pass
