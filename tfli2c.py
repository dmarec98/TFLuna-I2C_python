"""
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
 # Package: tfli2c
 # Developer: Bud Ryerson
 # Inception: 10 JUL 2021
 # Version:   0.0.1
 # Described: Python module for the Benewake TF-Luna Lidar sensor
 #            configured for I2C communication mode
 # Last Work: 17 JUL 2021
 #
 # Default settings for this module are:
 #    0x10  -  slave device address
 #      4   -  host I2C port address
 #
 #  begin( addr, port)
 #  sets the variables `tflAddr` and `tflPort`
 #  for use by the rest of the module and
 #  sets device sampling to `trigger` mode
 #
 #  getData()
 #  reads first six registers of device and
 #  sets the value of three variables
    -  dist : distance measured by the device, in cm
    -  flux : signal strength, quality or confidence
              If flux value too low, an error will occur.
    -  temp : temperature of the chip in 0.01 degrees C

      Returns true, if no error occurred.
      If false, error is defined by a status code,
      which can be displayed using `printStatus()` function.
 #
 #  There are several explicit commands. In general,
 #  the `set` commmands require a follow-on `saveSettings`
 #  and `softReset` commands.
 #
=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
"""

import time
from smbus import SMBus


class Reg:
    """
    - - -  Register Names, Hex Address Numbers  - - -
    - - - -  and whether Read, Write or both  - - - -
    """
    TFL_DIST_LO = 0x00  # R centimeters
    TFL_DIST_HI = 0x01  # R
    TFL_FLUX_LO = 0x02  # R arbitray units
    TFL_FLUX_HI = 0x03  # R
    TFL_TEMP_LO = 0x04  # R degrees Celsius
    TFL_TEMP_HI = 0x05  # R
    TFL_TICK_LO = 0x06  # R milliseconds
    TFL_TICK_HI = 0x07  # R
    TFL_ERR_LO = 0x08  # R
    TFL_ERR_HI = 0x09  # R
    TFL_VER_REV = 0x0A  # R revision
    TFL_VER_MIN = 0x0B  # R minor release
    TFL_VER_MAJ = 0x0C  # R major release

    TFL_PROD_CODE = 0x10  # R - 14 byte serial number

    TFL_SAVE_SETTINGS = 0x20  # W -- Write 0x01 to save
    TFL_SOFT_RESET = 0x21  # W -- Write 0x02 to reboot.
    # Lidar not accessible during few seconds,
    # then register value resets automatically
    TFL_SET_I2C_ADDR = 0x22  # W/R -- Range 0x08,0x77.
    # Must save and reboot to take effect.
    TFL_SET_MODE = 0x23  # W/R -- 0-continuous, 1-trigger
    TFL_TRIGGER = 0x24  # W  --  1-trigger once
    TFL_DISABLE = 0x25  # W/R -- 0-enable, 1-disable
    TFL_FPS_LO = 0x26  # W/R -- lo byte
    TFL_FPS_HI = 0x27  # W/R -- hi byte
    TFL_SET_LO_PWR = 0x28  # W/R -- 0-normal, 1-low power
    TFL_HARD_RESET = 0x29  # W  --  1-restore factory settings

    # - -   Error Status Condition definitions - -
    TFL_READY = 0  # no error
    TFL_SERIAL = 1  # serial timeout
    TFL_HEADER = 2  # no header found
    TFL_CHECKSUM = 3  # checksum doesn't match
    TFL_TIMEOUT = 4  # I2C timeout
    TFL_PASS = 5  # reply from some system commands
    TFL_FAIL = 6  # "
    TFL_I2CREAD = 7
    TFL_I2CWRITE = 8
    TFL_I2CLENGTH = 9
    TFL_WEAK = 10  # Signal Strength ≤ 100
    TFL_STRONG = 11  # Signal Strength saturation
    TFL_FLOOD = 12  # Ambient Light saturation
    TFL_MEASURE = 13
    TFL_INVALID = 14  # Invalid operation sent to sendCommand()


class Lidar:

    status = 0  # error status code
    dist = 0  # distance to target
    flux = 0  # signal quality or intensity
    temp = 0  # internal chip temperature

    # Timeout Limits for various functions
    TFL_MAX_READS = 20  # readData() sets SERIAL error
    MAX_BYTES_BEFORE_HEADER = 20  # getData() sets HEADER error
    MAX_ATTEMPTS_TO_MEASURE = 20

    tflAddr = 0x10  # TFLuna I2C device address
    # Range: 0x08 to 0x77
    tflPort = 4  # Raspberry Pi I2C port number

    # 4 = /dev/i2c-4, GPIO 8/9, pins 24/21

    def __init__(self, addr, port):
        self.tflAddr = addr  # re-assign device address
        self.tflPort = port  # re-assign port number
        bus = SMBus(self.tflPort)
        bus.open(self.tflPort)  # Open I2C communication
        bus.write_quick(self.tflAddr)  # Short test transaction
        # Set device to single-shot/trigger mode
        bus.write_byte_data(self.tflAddr, Reg.TFL_SET_MODE, 1)
        bus.close()  # Close I2C communication

    def get_data(self):
        """
        - - - - - - - - - - - - - - - - - - - - - - - - - -
                    GET DATA FROM THE DEVICE
        - - - - - - - - - - - - - - - - - - - - - - - - - -
        Return `True`/`False` whether data received without
        error and set system status
        """
        ''' Get get three data values '''

        #  2. Get data from the device.
        bus = SMBus(self.tflPort)  # Open I2C communication
        # Trigger a one-shot data sample
        bus.write_byte_data(self.tflAddr, Reg.TFL_TRIGGER, 1)
        #  Read the first six registers
        frame = bus.read_i2c_block_data(self.tflAddr, 0, 6)
        bus.close()  # Close I2C communication

        #  3. Shift data from read array into the three variables
        self.dist = frame[0] + (frame[1] << 8)
        self.flux = frame[2] + (frame[3] << 8)
        self.temp = frame[4] + (frame[5] << 8)
        # Convert temp to degrees from hundredths
        self.temp = (self.temp / 100)
        # Convert Celsius to Fahrenheit
        # temp = ( temp * 9 / 5) + 32

        #  4.  Evaluate Abnormal Data Values
        if self.dist == -1:
            self.status = Reg.TFL_WEAK
            return False
        elif self.flux < 100:  # Signal strength < 100
            self.status = Reg.TFL_WEAK
            return False
        elif self.flux > 0x8000:  # Ambient light too strong
            self.status = Reg.TFL_FLOOD
            return False
        elif self.flux == 0xFFFF:  # Signal saturation
            self.status = Reg.TFL_STRONG
            return False

        #  5. Set Ready status and go home
        self.status = Reg.TFL_READY
        return True

    def save_settings(self):
        """
        - - - - -    SAVE SETTINGS   - - - - -
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_SAVE_SETTINGS, 1)
        bus.close()

    def soft_reset(self):
        """- - - -   SOFT RESET aka Reboot  - - - -"""
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_SOFT_RESET, 2)
        bus.close()

    def hard_reset(self):
        """
        - - - -   HARD RESET to Factory Defaults  - - - -
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_HARD_RESET, 1)
        bus.close()

    def set_i2c_addr(self, addr_new):
        """
        - - - - - -    SET I2C ADDRESS   - - - - - -
        Range: 0x08, 0x77.  Must be followed by
        `saveSettings()` and 'softReset()` commands
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_SET_I2C_ADDR, addr_new)
        bus.close()

    def set_enable(self):
        """
        - - - - -   SET ENABLE   - - - - -
        Turn on LiDAR
        Must be followed by Save and Reset commands
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_DISABLE, 0)
        bus.close()

    def set_disable(self):
        """
        - - - - -   SET DISABLE   - - - - -
        Turn off LiDAR
        Must be followed by Save and Reset commands
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_DISABLE, 1)
        bus.close()

    def set_mode_cont(self):
        """
        - - - - - -   SET CONTINUOUS MODE   - - - - - -
        Continuous ranging mode
        Must be followed by Save and Reset commands
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_SET_MODE, 0)
        bus.close()

    def set_mode_trig(self):
        """
        - - - - - -   SET TRIGGER MODE   - - - - - -
        Sample range only once when triggered
        Must be followed by Save and Reset commands
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_SET_MODE, 1)
        bus.close()

    def get_mode(self):
        """
        - - - - - -   GET TRIGGER MODE   - - - - - -
        Return mode type as a string.
        """
        bus = SMBus(self.tflPort)
        mode = bus.read_byte_data(self.tflAddr, Reg.TFL_SET_MODE)
        bus.close()
        if mode == 0:
            return 'continuous'
        else:
            return 'trigger'

    def set_trigger(self):
        """
        - - - - - -   SET TRIGGER   - - - - - =
        Trigger device to sample one time.
        """
        bus = SMBus(self.tflPort)
        bus.write_byte_data(self.tflAddr, Reg.TFL_TRIGGER, 1)
        bus.close()

    def set_frame_rate(self, fps):
        """
        - - - - -    SET FRAME RATE   - - - - - -
        Write `fps` (frames per second) to device
        Command must be followed by `saveSettings()`
        and `softReset()` commands.
        """
        bus = SMBus(self.tflPort)
        bus.write_word_data(self.tflAddr, Reg.TFL_FPS_LO, fps)
        bus.close()

    def get_frame_rate(self):
        """
        - - - - -    GET FRAME RATE   - - - - - -
        Return two-byte Frame Rate (frames per second) value
        """
        bus = SMBus(self.tflPort)
        fps = bus.read_word_data(self.tflAddr, Reg.TFL_FPS_LO)
        bus.close()
        return fps

    def get_time(self):
        """
        - - - -  GET DEVICE TIME (in milliseconds) - - -
        Return two-byte value of milliseconds since last reset.
        """
        bus = SMBus(self.tflPort)  # Open I2C communication
        tim = bus.read_word_data(self.tflAddr, Reg.TFL_TICK_LO)  # Get two bytes of time+
        bus.close()  # Close I2C communication
        return tim

    def get_prod_code(self):
        """
        - -  GET PRODUCTION CODE (Serial Number) - - -
        Return 14 ascii characters of serial number
        """
        prod_code = ''
        bus = SMBus(self.tflPort)  # Open I2C communication
        for i in range(14):  # Build the 'production code' string
            prod_code += chr(bus.read_byte_data(self.tflAddr, Reg.TFL_PROD_CODE + i))
        bus.close()  # Close I2C communication
        return prod_code

    def get_firmware_version(self):
        """
        - - - -    GET FIRMWARE VERSION   - - - -
        Return version as a string
        """
        version = ''
        bus = SMBus(self.tflPort)  # Open I2C communication
        #  Build the 'version' string
        version = \
            str(bus.read_byte_data(self.tflAddr, Reg.TFL_VER_MAJ)) + '.' + \
            str(bus.read_byte_data(self.tflAddr, Reg.TFL_VER_MIN)) + '.' + \
            str(bus.read_byte_data(self.tflAddr, Reg.TFL_VER_REV))
        bus.close()  # Close I2C communication
        return version  # Return the version string

    def print_status(self):
        """
        - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        - - - - -    The following are for testing purposes   - - - -
           They interpret error status codes and display HEX data
        - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        Called by either 'printFrame()' or 'printReply()'
        Print status condition either 'READY' or error type
        """

        print("Status: ", end='')
        if self.status == Reg.TFL_READY:
            print("READY", end='')
        elif self.status == Reg.TFL_SERIAL:
            print("SERIAL", end='')
        elif self.status == Reg.TFL_HEADER:
            print("HEADER", end='')
        elif self.status == Reg.TFL_CHECKSUM:
            print("CHECKSUM", end='')
        elif self.status == Reg.TFL_TIMEOUT:
            print("TIMEOUT", end='')
        elif self.status == Reg.TFL_PASS:
            print("PASS", end='')
        elif self.status == Reg.TFL_FAIL:
            print("FAIL", end='')
        elif self.status == Reg.TFL_I2CREAD:
            print("I2C-READ", end='')
        elif self.status == Reg.TFL_I2CWRITE:
            print("I2C-WRITE", end='')
        elif self.status == Reg.TFL_I2CLENGTH:
            print("I2C-LENGTH", end='')
        elif self.status == Reg.TFL_WEAK:
            print("Signal weak", end='')
        elif self.status == Reg.TFL_STRONG:
            print("Signal saturation", end='')
        elif self.status == Reg.TFL_FLOOD:
            print("Ambient light saturation", end='')
        else:
            print("OTHER", end='')
        print()


# If this module is executed by itself
if __name__ == "__main__":
    print("tfli2c - This Python module supports the Benewake",
          "TFLuna Lidar device in I2C mode.")
