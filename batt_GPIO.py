import RPi.GPIO as GPIO
from time import sleep

#23 and 24 battery cells
#8 battery is charging
#25 battery is charged
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

try:
    print("READING BATTERY SENSORS..." + bcolors.ENDC) 

    while True:
        if GPIO.input(23) or GPIO.input(24):
            print(bcolors.BOLD + bcolors.FAIL + "BATTERY CRITICALLY LOW :O" + bcolors.ENDC)
        if GPIO.input(25) and GPIO.input(8):
            pass
        else:
            if GPIO.input(25):
                print(bcolors.BOLD + bcolors.OKGREEN+ "BATTERY CHARGED COMPLETED :)" + bcolors.ENDC)
            if GPIO.input(8):
                print(bcolors.WARNING + "BATTERY IS CHARGING" + bcolors.ENDC)
        sleep(0.4)

finally:
    GPIO.cleanup()

