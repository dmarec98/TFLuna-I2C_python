# tfli2c
### A python module for the Benewake TFLuna LiDAR distance sensor in I2C mode

The **TFLuna** in I2C communication mode is unique among the Benewake family of LiDAR products.

First, the communications mode (UART/U2C) is set by the voltage level of Pin #5 rather than a command. Second, the internal registers of the device can be addressed directly. And third, Benewake advises that sampling data continuously without using the Pin #6 "data ready" signal is unreliable.  For simplicity and accuracy, this module sets the device to Trigger (One-Shot) Mode during initialization and sends a Trigger command before each call to read the data registers.  (The use of Trigger Mode also significantly reduces power consumption.)

This library is *not compatible* with any other Benewake LiDAR device in I2C mode. However, in serial (UART) mode, the **TFLuna** is largely compatible with the **TFMini-Plus** and is therefore able to use that module, `tfmplus.py`, for Raspberry Pi and other python projects.

This module requires the python **smbus** or **smbus2** module to be installed.  Additional instances of the module can be imported to support additional devices.
<hr />

### Primary Functions

`begin( addr, port)` sends I2C address and port parameters, tests communication, switches the device from its default Continuous Mode to the One-Shot or Trigger Mode, and returns a boolean result.
 
`getData()` reads the first six registers of the device and sets the value of three variables:
<br />&nbsp;&nbsp;&#8211;&nbsp; `dist` Distance to target in centimeters. Range: 0 to 1200
<br />&nbsp;&nbsp;&#8211;&nbsp; `flux` Strength or quality of return signal or error. Range: -1 and 0 to 32767
<br />&nbsp;&nbsp;&#8211;&nbsp; `temp` Temperature in quarter degrees of Celsius. Range: -25.00°C to 125.00°C<br />
  The function returns a boolean value and sets a one byte `status` code based on various data values from the device.<br />
  EXAMPLE: If ```flux < 100``` then device sets  ```dist = -1``` and the function sets ```status = TFL_WEAK``` and returns ```False```.<br />
  The function ```printStatus()```, if called, will display ```"Signal weak"```.

A variety of other commands are explicitly defined and  may be sent individually and as necessary.  They are broadly separated into "set" commands that modify device register values and and "get" commands that examine register values.
<hr />

### Explicit commands:
<br />&#8211;&nbsp;&nbsp; `saveSettings()` - save register changes
<br />&#8211;&nbsp;&nbsp; `softReset()` - reset, reboot and restart
<br />&#8211;&nbsp;&nbsp; `hardReset()` - restore factory defaults
<br />&#8211;&nbsp;&nbsp; `setI2Caddr( addrNew)` - send value of new I2C address: `0x08` to `0x77`
<br />&#8211;&nbsp;&nbsp; `setEnable()` - turn ON device light source
<br />&#8211;&nbsp;&nbsp; `setDisable()` - turn OFF device light source
<br />&#8211;&nbsp;&nbsp; `setModeCont()` - set device to sample continmuously at frame rate
<br />&#8211;&nbsp;&nbsp; `setModeTrig()` - set device to sample once when triggered
<br />&#8211;&nbsp;&nbsp; `getMode()` - returns string of mode type: 'continuous' or 'trigger'
<br />&#8211;&nbsp;&nbsp; `setTrigger()` - trigger device to sample one time
<br />&#8211;&nbsp;&nbsp; `setFrameRate( fps)` - set device Frame-Rate in frames per second
<br />&#8211;&nbsp;&nbsp; `getFrameRate()` - return two-byte unsigned word of Frame-Rate in frames per second
<br />&#8211;&nbsp;&nbsp; `getTime()` - return two-byte unsigned word of device clock in milliseconds
<br />&#8211;&nbsp;&nbsp; `getProdCode()` - return 14 character string of product serial number
<br />&#8211;&nbsp;&nbsp; `getFirmwareVersion()`  - return string of version number

<hr>

In **I2C** mode, the TFMini-Plus functions as an I2C slave device.  The default address is `0x10` (16 decimal), but is user-programmable by sending the `setI2Caddr( addrNew)` command and a parameter in the range of `0x07` to `0x77` (7 to 119).  The new address requires a `softReset()` command to take effect.  A `hardReset()` command (Restore Factory Settings) will reset the device to the default address of `0x10`.

Some commands that modify internal parameters are processed within 1 millisecond.  But some commands that require the MCU to communicate with other chips may take several milliseconds.  And some commands that erase the flash memory of the MCU, such as `Save_Settings` and `Hard_Reset`, may take several hundred milliseconds.

Frame-rate and most other parameter changes should be followed by a `Save_Settings` command or the values may be lost when power is removed.  With the TFLuna, commands are available to examine the value of various device parameters such as frame rate, trigger mode, power mode, threshold values, internal timer, error and production code.

<hr>

Also included in the package are:
<br />&nbsp;&nbsp;&#9679;&nbsp; In the `tests` folder: An example Python sketch, `tfli2c_test.py`, and a simplified version of the same code, `tfli2c_simple.py`.
<br />&nbsp;&nbsp;&#9679;&nbsp; In the `docs` folder: A recent copy of the manufacturer's Product Manual.

All of the code for this Library is richly commented to assist with understanding and in problem solving.


