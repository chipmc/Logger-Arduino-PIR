#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# All rights reserved
# http://embedxcode.weebly.com
#
# Last update: Mar 06, 2017 release 7.2.0


# Libraries
# ----------------------------------
# Declare application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring
# and user's libraries used 
# Short-listing libraries speeds-up building
# Typing = 0 takes none (default for application and user)
# Otherwise, leaving empty considers all (default for local)

# List application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring libraries here
# default = 0 = none
#
# ARDUINO : AVR = EEPROM HID SoftwareSerial SPI Wire 
# ARDUINO : common = Bridge Esplora Ethernet Firmata GSM 
#	LiquidCrystal Robot_Control Robot_Motor RobotIRremote SD Servo 
#	SpacebrewYun Stepper Temboo TFT WiFi 
#
APP_LIBS_LIST = Wire EEPROM

# List user's libraries here 
# default = 0 = none
#
# USER = Adafruit_ADS1X15 Adafruit_AM2315 Adafruit_BLE_UART 
#	Adafruit_BLEFirmata Adafruit_BluefruitLE_nRF51 Adafruit_CC3000 
#	Adafruit_DotStar Adafruit_FONA Adafruit_FONA_OLD Adafruit_FONA_Teensy 
#	Adafruit_FRAM_I2C Adafruit_GFX_Library Adafruit_GPS Adafruit_INA219 
#	Adafruit_MAX31855 Adafruit_NeoPixel Adafruit_SSD1306 APA102 
#	arduino_865227 Bounce2 Bridge DHT DS1307RTC DS3232RTC DS3234 DSRTCLib 
#	EnableInterrupt FastLED Firmata fxos8700cq GPS HIH61XX HIH61XXCommander 
#	i2c_t3 Keyboard Keypad LGPRS LSM303 MAX17043 MicroView Mouse 
#	MQTT MQTTClient NewPing Phant PubSubClient RadioHead RFM69 
#	Robot_Control RTClib RunningAverage SdFat Servo SFE_CC3000 
#	SFE_MicroOLED SoftI2CMaster SpacebrewYun SPIFlash Temboo Time TimeLib 
#	Timezone TinyWireM Twitter U8glib Ubidots 
#
USER_LIBS_LIST = MAX17043 Adafruit_FRAM_I2C TimeLib DS3232RTC

# List local libraries here
# default = empty = all
#
LOCAL_LIBS_LIST = digitalWriteFast FRAMcommon

# List core, application and user's libraries to be excluded
# For example, WiFi may crash on Arduino 1.0.2, Esplora on Arduino 1.0.3, Firmata on Teensy 3.0,
#   OneWire on MPIDE 0023, HardwareSerial may conflict with MarlinSerial
#
#EXCLUDE_LIBS = Firmata WiFi Esplora OneWire Robot_Control Robot_Control/utility Robot_Motor


# Warning options
# ----------------------------------
# Contrary to errors, warnings don't stop compilation but they point at possible cause of errors.
# For example, unused variables with unused-variable.
#
# If 0, no warnings
# If empty, all warnings, same as WARNING_OPTIONS = all
# WARNING_OPTIONS = all no-missing-braces no-conversion-null no-pointer-arith
# WARNING_OPTIONS = unused-variable unused-function unused-label unused-value no-conversion-null no-pointer-arith
#
WARNING_OPTIONS = 0


# Applications path folder
# ----------------------------------
# Path to the folder where the IDEs are located
# By default, = /Applications
#
#APPLICATIONS_PATH = /Applications


# Serial console for Fast target
# ----------------------------------
# = 0 or = empty or comment: open serial console
# = 1: don't open serial console
#
NO_SERIAL_CONSOLE =


# Parameters
# ----------------------------------
# Xcode takes BOARD_TAG and BOARD_PORT from the .xcconfig file
# For other IDEs than Xcode,

# BOARD_TAG is required
# If not defined, error BOARD_TAG not defined
#
#BOARD_TAG  = uno

# BOARD_PORT is optional
# If not defined, BOARD_PORT = /dev/tty.usb* (default)
#
BOARD_PORT = /dev/cu.usbserial-DN01JSG8

# Xcode takes SKETCHBOOK_DIR from preferences.txt
# If not defined, each IDE has its own SKETCHBOOK_DIR (default)
# To share the same SKETCHBOOK_DIR along all IDEs, define
#
#SKETCHBOOK_DIR =

# SERIAL_BAUDRATE for the serial console, 9600 by default
# Uncomment and specify another speed
#
#SERIAL_BAUDRATE = 9600

# OPTIMISATION for the optimisation and debugger levels
# Uncomment and specify another options
#
#OPTIMISATION    = -Os -g3


# Miscellaneous
# ----------------------------------
#
EMBEDXCODE_EDITION = embedXcode+ Arduino

# Manage path with space in the name
CURRENT_DIR_SPACE := $(shell pwd)
CURRENT_DIR       := $(shell echo '$(CURRENT_DIR_SPACE)' | sed 's/ /\\\ /g')

# C-compliant project name
PROJECT_NAME_AS_IDENTIFIER = Logger_Arduino_PIR
SKETCH_EXTENSION   = ino

# Include only current variant of selected platform
SELECTED_RESOURCES = true

# For distribution, keep main cpp, false or true
KEEP_MAIN_CPP = false

# For building, show details, false or true
HIDE_NUMBER = false
HIDE_COMMAND = false

# Local makefiles
#MAKEFILE_PATH     := $(CURRENT_DIR)/Makefiles
#UTILITIES_PATH    := $(CURRENT_DIR)/Utilities

# Remote makefiles
EMBEDXCODE_TOOLS   = $(HOME)/Library/embedXcode/Tools
MAKEFILE_PATH      = $(EMBEDXCODE_TOOLS)/Makefiles
UTILITIES_PATH     = $(EMBEDXCODE_TOOLS)/Utilities

# Local builds
#BUILDS_PATH_SPACE := $(CURRENT_DIR_SPACE)/Builds
#BUILDS_PATH       := $(CURRENT_DIR)/Builds

# Remote builds
BUILDS_PATH       := $(BUILT_PRODUCTS_DIR)
BUILDS_PATH_SPACE := $(BUILDS_PATH)

include $(MAKEFILE_PATH)/Step1.mk

