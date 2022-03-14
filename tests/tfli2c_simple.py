'''=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# File Name: tfli2c_simple.py
# Inception: 14 JUL 2021
# Developer: Bud Ryerson
# Version:   0.0.1
# Last work: 17 JUL 2021

# Description: A simple Raspberry Pi Python script
# to test the Benewake TFLuna in I2C mode using
# the 'tfli2c' module.

# NOTE:
#   I2C(1) is default RPi I2C port, but used by real-time clock.
#   Other I2C Ports are initialized in 'boot/config.txt' file
#   I2C(0) = GPIO0 Pin 27 SDA, GPIO1 Pin 28 SCL
#   I2C(4) = GPIO8 Pin 24 SDA, GPIO9 Pin 21 SCL
#
# Press Ctrl-C to break the loop
#
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'''

import time  # Needed for 'sleep' function
import sys
import tfli2c as tfl  # Import `tfli2c` module v0.0.1

#   - - -  Set I2C Port and Address numbers  - - - - - - - -
i2c_addr = 0x10  # Device address in Hex, Decimal 16
i2c_port = 1  # I2C(1), /dev/i2c-1, GPIO 2/3, pins 3/5

#  - - -  Initalize module and device - - -
try:
    connexion1 = tfl.Lidar(i2c_addr, i2c_port)
    print("Ready")
except OSError:
    print("Not ready")
    sys.exit()  # quit the program if not ready

#  - - - -  Loop until an exception occurs  - - - - -
while True:
    try:
        time.sleep(0.05)  # Delay 50ms for 20Hz loop-rate
        connexion1.get_data()  # Get tfl data
        print(connexion1.dist)  # display distance
    #
    #  Use control-C to break loop
    except KeyboardInterrupt:
        print('Keyboard Interrupt')
        break
    #
    '''
    #  Catch all other exceptions
    except Exception:
        eType = sys.exc_info()[0]  # Return exception type
        print(eType)
        break
    '''
#
print("That's all folks!")  # Say "Goodbye!"
sys.exit()  # Clean up the OS and exit.
#
#  - - - - -  Everything ends here  - - - - - -
