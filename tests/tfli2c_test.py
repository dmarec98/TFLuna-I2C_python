"""
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# File Name: tfli2c_test.py
# Inception: 12 JUL 2021
# Developer: Bud Ryerson
# Version:   0.0.1
# Last work: 19 JUL 2021

# Description: Python script to test the Benewake TFLuna
# time-of-flight Lidar ranging sensor in I2C mode
# using the 'tfli2c' module in development.

# Default settings for the TFLuna are a 0x10 I2C address
# and a 100Hz measurement frame rate. The device will return
# three measurements:
#   Distance in centimeters,
#   Signal strength in arbitrary units and
#   Temperature in quarters of a degree centigrade

# 'begin( port, address)' must be called to set the I2C host
#  port and device address numbers.  It tests the existence
#  of the port and address, and returns a boolean result.
#  Continuous ranging is not recommended in I2C mode and so
#  this function also sets the device to single sample mode.

# NOTE:  Additional instances of this module (tfl2, tfl3,
# etc) should be imported to control additional devices.

# 'getData()' sets module variables for dist(distance),
#  flux (signal strength) and temp(temperature in Centigrade).
#  It returns a boolean value and sets a one byte error status
#  code based on data values from the device.
# EXAMPLE: If flux less than 100 then dist is set to -1,
#  getData() returns `False`, and status is set to "Signal weak".

# Various other commands are sent individually as necessary
# Commands are defined in the module's list of commands.
# Parameters, if any, are entered directly (0x10, 250, etc.)

# NOTE:
#  I2C(1) is default RPi I2C port, but used by real-time clock.
#  Other I2C Ports are initialized in the 'boot/config.txt' file
#  I2C(0) = GPIO0 Pin 27 SDA, GPIO1 Pin 28 SCL
#  I2C(4) = GPIO8 Pin 24 SDA, GPIO9 Pin 21 SCL
#
# Press Ctrl-C to break the loop
#
# 'tmli2c' does not work in Windows because required
# 'smbus' module only works in Linux/Raspian/MacOS
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
"""

import time
import sys
import tfli2c as tfl    # Import `tfli2c` module v0.0.1

# I2CPort = 0     # I2C(0), /dev/i2c-0, GPIO 0/1, pins 27/28
i2c_port = 1      # I2C(4), /dev/i2c-4, GPIO 8/9, pins 24/21
i2c_addr = 0x10   # Device address in Hex, Decimal 16

# - - - -  Set and Test I2C communication  - - - -
#  This function is needed to set the I2C port and
#  address values, and to test those settings.
# - - - - - - - - - - - - - - - - - - - - - - - - -
try:
    connexion1 = tfl.Lidar(i2c_addr, i2c_port)
    print("I2C mode: ready")
except OSError:
    print("I2C mode: not ready")
    sys.exit()  # quit the program if I2C bus not ready

#  - - - - - - - - - - - - - - - - - - - - - - - -'''

#  - - - - - -  Miscellaneous commands  - - - - - - -
#  These commands are for example only.
#  They are not necessary for this sketch to run.
#  There are many more commands available.
# - - - - - - - - - - - - - - - - - - - - - - - - -
#
#  - - Perform a system reset - - - - - - - -
print("System reset: ", end='')
connexion1.soft_reset()
time.sleep(0.5)  # allow 500ms for reset to complete
print("complete")
#
#  - - Get and Display the firmware version - - - - - - -
print("Firmware version: " + connexion1.get_firmware_version())
#
#  - - Get and display Serial Number - - - - - - -
print("Production Code: " + connexion1.get_prod_code())
#
#  - -  Set and display Trigger Mode  - - - - - -
connexion1.set_mode_trig()
print("Sample Mode: " + connexion1.get_mode())
#
#  - - -  Set Frame Rate to 20fps  - - - - -
#  - - -  and display Frame Rate   - - - - -
connexion1.set_frame_rate(20)
print("Sample Rate: " + str(connexion1.get_frame_rate()))
#  - - - - - - - - - - - - - - - - - - - - - - - -
#
time.sleep(0.5)     # Wait half a second.
#  - - - - - -  miscellaneous commands ends here  - - - - - - -

#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#  - - - - - -  the main program loop begins here  - - - - - - -
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#  If system error, wait two seconds and restart.
#  Quit after third attempt.
tfAttempt = 0
#
while tfAttempt < 3:
    try:
        #  Loop until exception occurs
        while True:
            time.sleep(0.047)   # Add 47ms delay for 20Hz loop-rate
            #  - - - - - - - - - - - - - - - - - - - - - - - - -
            #  This line of code:
            #      print( f"{value:0{padding}}", end = '')
            #  formats 'value' as a decimal number padded with 0s
            #  to the length of 'padding' and no CR/LF at the end.
            #  - - - - - - - - - - - - - - - - - - - - - - - - -
            #  Display three main data values from the device.
            if connexion1.get_data():
                # Display distance in centimeters,
                print(f"Dist:{connexion1.dist:{4}}cm", end=" | ")
                # display signal-strength or quality,
                print(f"Flux:{connexion1.flux:{6}d}", end=" | ")
                # and display temperature in Centigrade.
                print(f"Temp:{connexion1.temp:{3}}°C")
            else:                  # If the command fails...
                connexion1.print_status()  # display the error status
    #
    #  Use control-C to break loop.
    except KeyboardInterrupt:
        print('Keyboard Interrupt')
        break
    #
    #  Catch all other exceptions.
    except OSError:
        eType = sys.exc_info()[0]  # Return exception type
        print(eType)
        tfAttempt += 1
        print("Attempts: " + str(tfAttempt))
        time.sleep(2.0)     # Wait two seconds and retry.
#
print("That's all folks!")  # Say "Goodbye!"
sys.exit()                  # Clean up the OS and exit.
#
# - - - - - -  the main program loop ends here  - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
