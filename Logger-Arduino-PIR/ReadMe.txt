
  Logger-Arduino-PIR
  Project
  ----------------------------------
  Developed with embedXcode

  Project Logger-Arduino-PIR
  Created by Charles McClelland on 3/10/17
  Copyright © 2017 Charles McClelland
  Licence GNU General Public Licence

/*

Acknolwedgemetns - I have benefitted greatly from the following contributions.
- RTClib from Adafruit
- MAX1704 Library and code from Maxim
- Triembed.org - great group who answered my questions and showed me the path to this version

License - BSD Release 3

Hardware setup:
- Sparkfun Arduino Pro Mini - 3.3V / 8Mhz
- **Important** For the Watchdog Timer Functions to work, you need to reflash the Arduino with the MiniCore Bootloader
- For Information, go here: https://github.com/MCUdude/MiniCore
- PIR Sensor with 3.3V and Signal on INT2-D3
- INT2 - D3 - PIR Sensor interrupt - Active high
- INT1 - D2  - Not currently used
- TI FRAM Chip on i2c Bus
- DS3231 RTC Module on the i2c bus
- Indicator LED on pin 4

EEPROM Memory Map
0   Monthly Offset (value 1-12)
1-12 Monthly reboot counts for Jan (1) through December (12)


FRAM Memory Map v7
Memory Map - 256kb or 32kB divided into 4096 words - the  first one is reserved
Byte     Value
The first word is for system data
0        Memory Map Version (this program expects the value #defined in VERSIONNUMBER)
1        Park Opens  (24 hour clock)
2        Park Closes (24 hour clock)
3        Monthly Reboot Count - System Health
4        Daily Count Pointer
5-5      Current Hourly Count Pointer (16-bit number)
7        Control Register  (8 - 5 Reserved, 4- LEDs, 3-Start / Stop Test, 2-Warm Up, 1-LEDs)
The second word is for storing the current count data
8-9      Current Hourly Count (16-bit)
10-11    Current Daily Count (16-bit)
12-15    EPOCH Time when last counts recorded (32-bits)
Words 3-30 are 28 days worth of daily counts - if this changes - need to change #offsets and DAILYCOUNTNUMBER
0        Month
1        Day
2 - 3    Daily Count (16-bit)
4        Daily Battery Level
5        Reserved
6        Reserved
7        Reserved
Words from 31 to the end of the memory store hourly data
0 - 3    EPOCH Time
4 - 5    Hourly count (16-bit)
6        Houlry Battery Level
7        Reserved

The park is open for an average of 12 hours per day so about 340 days of hourly data on a 256k chip




  References
  ----------------------------------



  embedXcode
  embedXcode+
  ----------------------------------
  Embedded Computing on Xcode
  Copyright © Rei VILO, 2010-2017
  All rights reserved
  http://embedXcode.weebly.com

