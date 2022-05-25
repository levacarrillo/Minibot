#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int16
from minibot_msgs.srv import GetBattPerc, GetBattPercResponse

#GPIO 23 and 24 are battery cells
#GPIO 8 battery charging
#GPIO 25 battery charged
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN)
GPIO.setup(24, GPIO.IN)
GPIO.setup(25, GPIO.IN)
GPIO.setup(8,  GPIO.IN) 

batt_percentage = 0

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

def callback(data):
    global batt_percentage

    min_perc = 20#%
    max_perc = 99#%
    min_analog = 418
    max_analog = 528

    batt_percentage = min_perc + (data.data - min_analog) * (max_perc - min_perc) / (max_analog - min_analog)

    if(batt_percentage < 0): batt_percentage = 0 
    if(batt_percentage > 99): batt_percentage = 99 
    #print(data.data, batt_percentage)

def handle_get_bat_perc(req):
    return GetBattPercResponse(batt_percentage) 

def battery_reader():
    rospy.init_node('battery_reader', anonymous=True)
    rate = rospy.Rate(0.5)
    print("Starting battery_reader node by Luis Nava...")

    rospy.Subscriber("/battery_data", Int16, callback)
    s = rospy.Service('battery_perc', GetBattPerc, handle_get_bat_perc)

    try:

        while not rospy.is_shutdown():
            if GPIO.input(23) or GPIO.input(24):
                print(bcolors.BOLD + bcolors.FAIL + "BATTERY CRITICALLY LOW :O" + bcolors.ENDC + " -- Battery at->" + bcolors.BOLD + str(batt_percentage) + "%" + bcolors.ENDC)
            if GPIO.input(25) and GPIO.input(8):
                pass
            else:
                if GPIO.input(25):
                    print(bcolors.BOLD + bcolors.OKGREEN + "BATTERY CHARGED COMPLETED :)" + bcolors.ENDC)
                if GPIO.input(8):
                    print(bcolors.WARNING + "BATTERY IS CHARGING" + bcolors.ENDC + " -- Battery at->" + bcolors.BOLD + str(batt_percentage) + "%" + bcolors.ENDC)
                
            rate.sleep()
            
            try:
                msg = rospy.wait_for_message("/battery_data", Int16, timeout=1)
            except:
                pass

            #print("Battery at->" + str(batt_percentage) + "%")

    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        battery_reader()
    except rospy.ROSInterruptException:
        pass
