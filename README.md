# ISL9402 clone
script for quick cloning of ISL9402 parameters in production. ISL9402_clone.py -w.
a master file with settings can be made with the configured BMS key ISL9402_clone.py -r or use ISL94202_EEPROM_Form.xlsm.
For example, the setting of some registers with temperature is implemented. The temperature is set in degrees Celsius ISL9402_clone.py -set DOT -20.
You cannot use the Renesas software support. You will find xlsm, VB6 in 2022 is not funny. Only windows and old Microsoft Excel.
As necessary, settings for other registers will be added.

# Features
Windows and Linux support.
in Linux Kernel driver.
in Windows libusb.
you can detect i2c addresses in windows. py-i2cdetect.py

# References:
  # Win & Linux

https://www.adafruit.com/product/2000

https://github.com/harbaum/I2C-Tiny-USB

  # Pyton

https://www.fischl.de/i2c-mp-usb/

  # only linux

https://www.adafruit.com/product/4382

https://github.com/daniel-thompson/i2c-star
