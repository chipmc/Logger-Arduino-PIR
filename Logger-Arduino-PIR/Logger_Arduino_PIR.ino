///
/// @mainpage	Logger-Arduino-PIR
///
/// @details	Arduino Side (miniCore) - PIR Sensor
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		3/10/17 9:52 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Logger_Arduino_PIR.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		3/10/17 9:52 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE



// Set parameters
// There are some new pin assignments when using the new v10 board - default for the PIR sensor since we can turn it off
#define V10BOARD 1
#if V10BOARD                    // These are the pin assignments for the v9 board
#define ALARMPIN 3         // This one will be used for the RTC Alarm in v9
#define INT2PIN 2         // This is the interrupt pin that registers taps
#define INTNUMBER 0         // so I don't have to use the lookup function
#define PIRPIN 5            // This is a pin which connects to the i2c header - future use
#define I2CPWR 8            // Turns the i2c port on and off
#define RESETPIN 16         // This a modification using a bodge wire
#define TALKPIN 14           // This is the open-drain line for signaling i2c mastery (A0 on the Uno is 14)
#define THE32KPIN 15      // This is a 32k squarewave from the DS3231 (A1 on the Uno is 15)
#else                      // These are the pin assignments for the v8b board
#define SENSORPIN 2         // Not used now but wired for future use
#define PIRPIN 3         // This is the interrupt pin for the PIR Sensor, Active High, Push-Pull
#define ALARMPIN 5         // This is the pin with the RTC Alarm clock - not used on Arduino side
#define I2CPWR 8            // Turns the i2c port on and off
#define RESETPIN 16         // This a modification using a bodge wire
#define TALKPIN 14           // This is the open-drain line for signaling i2c mastery (A0 on the Uno is 14)
#define THE32KPIN 15      // This is a 32k squarewave from the DS3231 (A1 on the Uno is 15)
#endif

//Time Period Definitions - used for debugging
#define HOURLYPERIOD hour(t)   // Normally hour(t) but can use minute(t) for debugging
#define DAILYPERIOD day(t) // Normally day(t) but can use minute(t) or hour(t) for debugging

//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 7             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2               // First word of daily counts
#define HOURLYOFFSET 30             // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28         // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0             // Memory Locations By Name not Number
#define PARKOPENSADDR 0x1           // When does the park open
#define PARKCLOSESADDR 0x2          // when does the park close
#define MONTHLYREBOOTCOUNT 0x3      // This is where we store the reboots - indication of system health
#define DAILYPOINTERADDR 0x4        // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7         // This is the control register acted on by both Simblee and Arduino
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count
#define CURRENTDAILYCOUNTADDR 0xA   // Current Daily Count
#define CURRENTCOUNTSTIME 0xC       // Time of last count
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2          // Count is a 16-bt value
#define DAILYBATTOFFSET 4           // Where the battery charge is stored
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// LED Pin Value Variables
#define REDLED 6                    // led connected to digital pin 4
#define YELLOWLED 4                 // The yellow LED
#define LEDPWR 7                    // This pin turns on and off the LEDs
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "1.0.0"



// Include application, user and local libraries
#include <avr/sleep.h>          // For Sleep Code
#include <avr/power.h>          // Power management
#include <avr/wdt.h>            // Watchdog Timer
#include <EEPROM.h>
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include <Wire.h>               //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <TimeLib.h>            //http://www.arduino.cc/playground/Code/Time
#include "DS3232RTC.h"          //http://github.com/JChristensen/DS3232RTC
#include "Adafruit_FRAM_I2C.h"  // Library for FRAM functions
#include "FRAMcommon.h"         // Where I put all the common FRAM read and write extensions


// Prototypes
MAX17043 batteryMonitor;        // Initialize the Fuel Gauge

// Prototypes for General Functions
void StartStopTest(boolean startTest); // Since the test can be started from the serial menu or the Simblee - created a function
void BlinkForever(); // Ends execution
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation
void LogHourlyEvent(); // Log Hourly Event()
void LogDailyEvent(); // Log Daily Event()
void CheckForBump(); // Check for bump
void SetPinChangeInterrupt(byte Pin);  // Here is where we set the pinchange interrupt
void ClearPinChangeInterrupt(byte Pin);  // Here is where we clear the pinchange interrupt
void sleepNow();  // Puts the Arduino to Sleep
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay
int freeRam ();  // Debugging code, to check usage of RAM


// Prototypes for Date and Time Functions
void SetTimeDate(); // Sets the RTC date and time
void PrintTimeDate(time_t t); // Prints to the console
void toArduinoTime(time_t unixT); //Converts to Arduino Time for use with the RTC and program


// FRAM and Unix time variables
time_t t;
byte lastHour = 0;  // For recording the startup values
byte lastDate = 0;   // These values make sure we record events if time has lapsed
byte ParkOpens;             // When does the park open
byte ParkCloses;            // When does the park close
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts
int countTemp = 0;          // Will use this to see if we should display a day or hours counts

// Variables for the control byte
// Control Register  (8 - 7 Reserved, 6 - Simblee Reset, 5-Clear Counts, 4-Simblee Sleep, 3-Start / Stop Test, 2-Warmup, 1-LED state)
byte turnLedsOn = B00000001;    // Turn on the LEDs
byte turnLedsOff = B11111110;   // Turn off the LEDs
byte warmUpFlag = B00000010;    // Tells the Simblee it is warming up
byte clearWarmUpFlag = B11111101;   // Signals the warmup is complete
byte toggleStartStop = B00000100;
byte toggleSimbleeSleep = B00001000;
byte signalClearCounts = B00010000;
byte clearClearCounts = B11101111;
byte signalSimbleeReset = B00100000;        // Simblee will set this flag on disconnect
byte clearSimbleeReset = B11011111;         // The only thing the Arduino will do is clear the Simblee Reset Bit
byte controlRegisterValue;                  // Holds the current control register value
byte oldControlRegisterValue = B00000000;   // Makes sure we can detect a change in the register value
unsigned long lastCheckedControlRegister;   // When did we last check the control register
int controlRegisterDelay = 1000;            // Ho often will we check the control register


// PIR Sensor Variables
unsigned long warmUpTime = 1000;    // PIR Sensors need 45-60 seconds to warm up
volatile bool PIRInt = false;       // A flag for the PIR Interrupt
boolean countEnable = false;        // Need to count only once for each time the sensor triggers

// Battery monitor
float stateOfCharge = 0;            // stores battery charge level value

//Menu and Program Variables
unsigned long lastBump = 0;         // set the time of an event
bool ledState = false;              // variable used to store the last LED status, to toggle the light
bool refreshMenu = true;            //  Tells whether to write the menu
bool inTest = false;                // Are we in a test or not
bool LEDSon = true;                 // Are the LEDs on or off
int delaySleep = 3500;              // Wait until going back to sleep so we can enter commands
int menuChoice=0;                   // Menu Selection
int numberHourlyDataPoints;         // How many hourly counts are there
int numberDailyDataPoints;          // How many daily counts are there
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte bootcount = 0;                 // Counts reboots
int bootCountAddr = 0;              // Address for Boot Count Number


// Add setup code
void setup()
{
    Serial.begin(9600);                     // Initialize communications with the terminal
    Serial.print(".");
    Serial.println("");                     // Header information
    Serial.print(F("Connected Sensor PIR - release "));
    Serial.println(releaseNumber);
    Wire.begin();
    Serial.print(".");
    pinMode(REDLED, OUTPUT);            // declare the Red LED Pin as an output
    pinMode(YELLOWLED, OUTPUT);         // declare the Yellow LED Pin as as OUTPUT
    pinMode(LEDPWR, OUTPUT);            // declare the Power LED pin as as OUTPUT
    digitalWrite(LEDPWR, LOW);          // Turn on the power to the LEDs at startup for as long as is set in LEDsonTime
    pinMode(I2CPWR, OUTPUT);            // This is for V10 boards which can turn off power to the external i2c header
    digitalWrite(I2CPWR, HIGH);         // Turns on the i2c port
    pinMode(RESETPIN,INPUT);            // Just to make sure - if set to output, you can program the SIMBLEE
    pinMode(PIRPIN, INPUT);            // Set up the interrupt pins, they're set as active low with an external pull-up
    pinMode(THE32KPIN,INPUT);           // These are the pins tha are used to negotiate for the i2c bus
    pinMode(TALKPIN,INPUT);             // These are the pins tha are used to negotiate for the i2c bus
    Serial.print(".");
    enable32Khz(1); // turns on the 32k squarewave - to moderate access to the i2c bus
    Serial.print(".");
    
    TakeTheBus(); // Need the i2c bus for initializations
    Serial.print(".");
        if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
            Serial.println(F("Found I2C FRAM"));
        } else {
            Serial.println(F("No I2C FRAM found ... check your connections"));
            BlinkForever();
        }
    GiveUpTheBus(); // Done with i2c initializations Arduino gives up the bus here.
    Serial.println(".");
    
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
        Serial.print(F("FRAM Version Number: "));
        Serial.println(FRAMread8(VERSIONADDR));
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
        while (!Serial.available());
        switch (Serial.read()) {    // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'y':
                ResetFRAM();
                break;
            default:
                Serial.println(F("Cannot proceed"));
                BlinkForever();
        }
    }
    
    // Initialize the rest of the i2c devices
    ParkOpens = FRAMread8(PARKOPENSADDR);   // Get the Opening Time
    ParkCloses = FRAMread8(PARKCLOSESADDR); // Get the Park closing time
    TakeTheBus();
    batteryMonitor.reset();               // Initialize the battery monitor
    batteryMonitor.quickStart();
    setSyncProvider(RTC.get);              // Set up the clock as we will control it and the alarms here
    Serial.println(F("RTC Sync"));
    if (timeStatus() != timeSet) {
        Serial.println(F(" time sync fail!"));
        BlinkForever();
    }
    // We need to set an Alarm or Two in order to ensure that the Simblee is put to sleep at night
    if (ParkOpens > 12) ParkOpens = 12;     // Keep it in bounds
    if (ParkCloses > 23) ParkCloses = 23;   // Keep it in bounds
    else if (ParkCloses < 13) ParkCloses = 13;  // Keep it in bounds
    RTC.squareWave(SQWAVE_NONE);            //Disable the default square wave of the SQW pin.
    RTC.alarm(ALARM_1);                     // This will clear the Alarm flags
    RTC.alarm(ALARM_2);                     // This will clear the Alarm flags
    RTC.setAlarm(ALM1_MATCH_HOURS,00,00,ParkCloses,0); // Set the evening Alarm
    RTC.setAlarm(ALM2_MATCH_HOURS,00,00,ParkOpens,0); // Set the morning Alarm
    RTC.alarmInterrupt(ALARM_2, true);      // Connect the Interrupt to the Alarms (or not)
    RTC.alarmInterrupt(ALARM_1, true);
    GiveUpTheBus();
    
    FRAMwrite8(CONTROLREGISTER, toggleStartStop);       // Reset the control register and start the test
    
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    FRAMwrite8(CONTROLREGISTER, controlRegisterValue | turnLedsOn); // Turn on the LEDs at startup
    
    Serial.print(F("Sensor is warming up..."));
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    FRAMwrite8(CONTROLREGISTER, controlRegisterValue | warmUpFlag);  // Turn on the warm up flag
    while (millis() < warmUpTime);
    Serial.println(F("ready to go!"));
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    FRAMwrite8(CONTROLREGISTER, controlRegisterValue & clearWarmUpFlag);  // Turn off the warm up flag
    
    
    Serial.print(F("Monthly reboot count is "));
    bootCountAddr = EEPROM.read(0);             // Here is where we will track reboots by month - offset stored in 0 byte
    bootcount = EEPROM.read(bootCountAddr);     // Use the offset to get to this month's boot count
    bootcount++;                                // Increment the boot count
    Serial.print(bootcount);
    EEPROM.write(bootCountAddr, bootcount);     // Write it back into the correct spot
    FRAMwrite8(MONTHLYREBOOTCOUNT, bootcount); // Store in FRAM for access by Simblee in user interface
    Serial.print(F(" with a monthly offset of: "));
    TakeTheBus();
        t = RTC.get();
    GiveUpTheBus();
    bootCountAddr = month(t);                   // Boot counts are offset by month to reduce burn - in risk
    EEPROM.update(0, bootCountAddr);            // Will update the month if it has changed but only at reboot
    Serial.println(EEPROM.read(0));             // Print so we can see if code is working

    
    Serial.print(F("Free memory: "));
    Serial.println(freeRam());
    
}

void loop()
{
    if (refreshMenu)
    {
        refreshMenu = 0;
        Serial.println(F("Remote Trail Counter Program Menu"));
        Serial.println(F("0 - Display Menu"));
        Serial.println(F("1 - Display status"));
        Serial.println(F("2 - Set the clock"));
        Serial.println(F("3 - Set the Park Open Hour (24 hr format)"));
        Serial.println(F("4 - Set the Park Close Hour"));
        Serial.println(F("5 - Reset the counter"));
        Serial.println(F("6 - Reset the memory"));
        Serial.println(F("7 - Start / stop counting"));
        Serial.println(F("8 - Dump hourly counts"));
        Serial.println(F("9 - Last 14 day's counts"));
        NonBlockingDelay(100);
    }
    if (Serial.available() >> 0) {      // Only enter if there is serial data in the buffer
        switch (Serial.read()) {          // Read the buffer
            case '0':
                refreshMenu = 1;
                break;
            case '1':   // Display Current Status Information
                Serial.print(F("Current Time:"));
                TakeTheBus();
                t = RTC.get();
                GiveUpTheBus();
                PrintTimeDate(t);  // Give and take the bus are in this function as it gets the current time
                TakeTheBus();
                stateOfCharge = batteryMonitor.getSoC();
                GiveUpTheBus();
                Serial.print(F("State of charge: "));
                Serial.print(stateOfCharge);
                Serial.println(F("%"));
                Serial.print(F("Park Opens at: "));
                Serial.println(FRAMread8(PARKOPENSADDR));
                Serial.print(F("Park Closes at: "));
                Serial.println(FRAMread8(PARKCLOSESADDR));
                Serial.print(F("Hourly count: "));
                Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
                Serial.print(F("Daily count: "));
                Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
                Serial.print(F("Free memory: "));
                Serial.println(freeRam());
                Serial.print(F("Reboots: "));
                Serial.println(FRAMread8(MONTHLYREBOOTCOUNT));
                break;
            case '2':     // Set the clock
                SetTimeDate();
                PrintTimeDate(t);
                Serial.println(F("Date and Time Set"));
                break;
            case '3':   // Set Park Open Time
                Serial.println(F("Enter hour park opens (0-24):"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                ParkOpens = Serial.parseInt();
                FRAMwrite8(PARKOPENSADDR,ParkOpens);
                Serial.print(F("Park Open Time Set to: "));
                Serial.println(ParkOpens);
                RTC.setAlarm(ALM2_MATCH_HOURS,00,00,ParkOpens,0); // Set the morning Alarm
                break;
            case '4':   // Set Park Close Time
                Serial.println(F("Enter hour park closes (0-24):"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                ParkCloses = Serial.parseInt();
                FRAMwrite8(PARKCLOSESADDR,ParkCloses);
                Serial.print(F("Park Close Time Set to: "));
                Serial.println(ParkCloses);
                RTC.setAlarm(ALM1_MATCH_HOURS,00,00,ParkCloses,0); // Set the evening Alarm
                break;
            case '5':  // Reset the current counters
                Serial.println(F("Counter Reset!"));
                FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);   // Reset Daily Count in memory
                FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);  // Reset Hourly Count in memory
                hourlyPersonCount = 0;
                dailyPersonCount = 0;
                Serial.println(F("Resetting Counters and Simblee"));
                pinMode(RESETPIN, OUTPUT);
                digitalWrite(RESETPIN, LOW);
                NonBlockingDelay(100);
                digitalWrite(RESETPIN, HIGH);
                pinMode(RESETPIN, INPUT);
                break;
            case '6': // Reset FRAM Memory
                ResetFRAM();
                break;
            case '7':  // Start or stop the test
                if (inTest == 0) {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop | controlRegisterValue);    // Toggle the start stop bit high
                    StartStopTest(1);
                }
                else {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop ^ controlRegisterValue);    // Toggle the start stop bit low
                    StartStopTest(0);
                    refreshMenu = 1;
                }
                break;
            case '8':   // Dump the hourly data to the monitor
                numberHourlyDataPoints = FRAMread16(HOURLYPOINTERADDR); // Put this here to reduce FRAM reads
                Serial.print("Retrieving ");
                Serial.print(HOURLYCOUNTNUMBER);
                Serial.println(" hourly counts");
                Serial.println(F("Hour Ending -   Count  - Battery %"));
                for (int i=0; i < HOURLYCOUNTNUMBER; i++) { // Will walk through the hourly count memory spots - remember pointer is already incremented
                    unsigned int address = (HOURLYOFFSET + (numberHourlyDataPoints + i) % HOURLYCOUNTNUMBER)*WORDSIZE;
                    countTemp = FRAMread16(address+HOURLYCOUNTOFFSET);
                    if (countTemp > 0) {
                        time_t unixTime = FRAMread32(address);
                        toArduinoTime(unixTime);
                        Serial.print(F(" - "));
                        Serial.print(countTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+HOURLYBATTOFFSET));
                        Serial.println(F("%"));
                    }
                }
                Serial.println(F("Done"));
                break;
            case '9':  // Download all the daily counts
                numberDailyDataPoints = FRAMread8(DAILYPOINTERADDR);        // Put this here to reduce FRAM reads
                Serial.println(F("Date - Count - Battery %"));
                for (int i=0; i < DAILYCOUNTNUMBER; i++) {                  // Will walk through the 30 daily count memory spots - remember pointer is already incremented
                    int address = (DAILYOFFSET + (numberDailyDataPoints + i) % DAILYCOUNTNUMBER)*WORDSIZE;      // Here to improve readabiliy - with Wrapping
                    countTemp = FRAMread16(address+DAILYCOUNTOFFSET);       // This, again, reduces FRAM reads
                    if (countTemp > 0) {                                    // Since we will step through all 30 - don't print empty results
                        Serial.print(FRAMread8(address));
                        Serial.print(F("/"));
                        Serial.print(FRAMread8(address+DAILYDATEOFFSET));
                        Serial.print(F(" - "));
                        Serial.print(countTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+DAILYBATTOFFSET));
                        Serial.println(F("%"));
                    }
                }
                Serial.println(F("Done"));
                break;
            default:
                Serial.println(F("Invalid choice - try again"));
        }
        Serial.read();  // Clear the serial buffer
    }
    if (millis() >= lastCheckedControlRegister + controlRegisterDelay) {
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        lastCheckedControlRegister = millis();
        if (controlRegisterValue ^ oldControlRegisterValue) // XOR - will give a positive value if there has been a change
        {
            oldControlRegisterValue = controlRegisterValue;  //   By resetting here - every clause below needs to leave the flag correctly set
            if ((controlRegisterValue & toggleStartStop) >> 2 && !inTest)
            {
                StartStopTest(1);  // If the control says start but we are stopped
            }
            else if (!((controlRegisterValue & toggleStartStop) >> 2) && inTest)
            {
                StartStopTest(0); // If the control bit says stop but we have started
            }
            else if (controlRegisterValue & signalClearCounts)
            {
                TakeTheBus();
                    t = RTC.get();
                GiveUpTheBus();
                hourlyPersonCount = 0;
                dailyPersonCount = 0;
                FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
                FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
                FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
                Serial.println(F("Current Counts Cleared"));
                controlRegisterValue &= clearClearCounts;
                FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
            }
            else if (controlRegisterValue & signalSimbleeReset)  // If the reset flag is set
            {
                Serial.println("Resetting the Simblee");
                if (!(controlRegisterValue & toggleSimbleeSleep)) // Only reset if the Simblee is awake
                {
                    pinMode(RESETPIN, OUTPUT);
                    digitalWrite(RESETPIN, LOW);
                    NonBlockingDelay(100);
                    digitalWrite(RESETPIN, HIGH);
                    pinMode(RESETPIN, INPUT);
                }
                FRAMwrite8(CONTROLREGISTER, controlRegisterValue & clearSimbleeReset);  // Reset the Simblee Sleep flag
            }
            else if (!(controlRegisterValue & turnLedsOn) && LEDSon)  // Flag says "off" but lights are on
            {
                digitalWrite(LEDPWR,HIGH);
                LEDSon = false; // This keeps us from entering this conditional once we have turned off the lights
                Serial.println(F("Turn off the LEDs"));
            }
            else if (controlRegisterValue & turnLedsOn && !LEDSon)    // Flag says "on" but lights are off
            {
                digitalWrite(LEDPWR,LOW);
                LEDSon = true; // This keeps us from entering this conditional once we have turned on the lights
                Serial.println(F("Turn on the LEDs"));
            }
        }
    }
    if (inTest == 1) {
        CheckForBump();
        if (millis() >= lastBump + delaySleep) {
            sleepNow();     // sleep function called here
        }
    }
}

void CheckForBump() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
    if(digitalRead(PIRPIN) && countEnable)
    {
        lastBump = millis();    // Reset last bump timer
        countEnable = false;    // Make sure we only count once for each trigger
        Serial.print("Detected");
        ledState = !ledState;
        digitalWrite(REDLED,ledState);
        PIRInt = false; // Reset the flag
        TakeTheBus();
            t = RTC.get();
        Serial.print(".");
        GiveUpTheBus();
        Serial.print(".");
        if (t == 0) return;     // This means there was an error in reading the real time clock - very rare in testing so will simply throw out this count
        if (HOURLYPERIOD != currentHourlyPeriod) {
            LogHourlyEvent();
        }
        Serial.print(".");
        if (DAILYPERIOD != currentDailyPeriod) {
            LogDailyEvent();
        }
        Serial.print(".");
        hourlyPersonCount++;                    // Increment the PersonCount
        FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
        Serial.print(".");
        dailyPersonCount++;                    // Increment the PersonCount
        FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
        Serial.print(".");
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        Serial.print(".");
        Serial.print(F("Hourly: "));
        Serial.print(hourlyPersonCount);
        Serial.print(F(" Daily: "));
        Serial.print(dailyPersonCount);
        Serial.print(F(" Reboots: "));
        Serial.print(bootcount);
        Serial.print(F("  Time: "));
        PrintTimeDate(t);
    }
    else if (!digitalRead(PIRPIN)) countEnable = true;      // Reset the count enable trigger
}

void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
{
    tmElements_t tm;
    TakeTheBus();
        t = RTC.get();                    // Gets the current time
    GiveUpTheBus();
    if (startTest) {
        inTest = true;
        currentHourlyPeriod = HOURLYPERIOD;   // Sets the hour period for when the count starts (see #defines)
        currentDailyPeriod = DAILYPERIOD;     // And the day  (see #defines)
        // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
        time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
        breakTime(unixTime, tm);
        lastHour = tm.Hour;
        lastDate = tm.Day;
        dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
        hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
        if (currentDailyPeriod != lastDate) {
            LogHourlyEvent();
            LogDailyEvent();
        }
        else if (currentHourlyPeriod != lastHour) {
            LogHourlyEvent();
        }
        Serial.println(F("Test Started"));
    }
    else {
        PIRInt = false;                 // Reset this flag in case interrupt just happened
        inTest = false;                 // Set the flag that indicates we are not in a test
        FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
        FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        hourlyPersonCount = 0;          // Reset Person Count
        dailyPersonCount = 0;           // Reset Person Count
        Serial.println(F("Test Stopped"));
    }
}

void LogHourlyEvent() // Log Hourly Event()
{
    tmElements_t timeElement;       // We will need to break down the current time
    time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
    breakTime(LogTime, timeElement);                    // Break the time into its pieces
    unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    LogTime -= (60*timeElement.Minute + timeElement.Second); // So, we need to subtract the minutes and seconds needed to take to the top of the hour
    FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
    FRAMwrite16(pointer+HOURLYCOUNTOFFSET,hourlyPersonCount);
    TakeTheBus();
    stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
    unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
    hourlyPersonCount = 0;                    // Reset and increment the Person Count in the new period
    currentHourlyPeriod = HOURLYPERIOD;  // Change the time period
    Serial.println(F("Hourly Event Logged"));
}

void LogDailyEvent() // Log Daily Event()
{
    tmElements_t timeElement;
    time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);// This is the last event recorded - this sets the daily period
    breakTime(LogTime, timeElement);
    int pointer = (DAILYOFFSET + FRAMread8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    FRAMwrite8(pointer,timeElement.Month); // The month of the last count
    FRAMwrite8(pointer+DAILYDATEOFFSET,timeElement.Day);  // Write to FRAM - this is the end of the period  - should be the day
    FRAMwrite16(pointer+DAILYCOUNTOFFSET,dailyPersonCount);
    TakeTheBus();
    stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+DAILYBATTOFFSET,stateOfCharge);
    byte newDailyPointerAddr = (FRAMread8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite8(DAILYPOINTERADDR,newDailyPointerAddr);
    dailyPersonCount = 0;    // Reset and increment the Person Count in the new period
    currentDailyPeriod = DAILYPERIOD;  // Change the time period
    Serial.println(F("Logged a Daily Event"));
}

void SetTimeDate()  // Function to set the date and time from the terminal window
{
    tmElements_t tm;
    Serial.println(F("Enter Seconds (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Second = Serial.parseInt();
    Serial.println(F("Enter Minutes (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Minute = Serial.parseInt();
    Serial.println(F("Enter Hours (0-23): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Hour= Serial.parseInt();
    Serial.println(F("Enter Day of the Month (1-31): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Day = Serial.parseInt();
    Serial.println(F("Enter the Month (1-12): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Month = Serial.parseInt();
    Serial.println(F("Enter the Year (e.g. 2017): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Year = CalendarYrToTm(Serial.parseInt());
    t= makeTime(tm);
    TakeTheBus();
    RTC.set(t);             //use the time_t value to ensure correct weekday is set
    setTime(t);
    GiveUpTheBus();
}

void PrintTimeDate(time_t t)  // Prints time and date to the console
{
    Serial.print(year(t), DEC);
    Serial.print('/');
    Serial.print(month(t), DEC);
    Serial.print('/');
    Serial.print(day(t), DEC);
    Serial.print(F(" "));
    Serial.print(hour(t), DEC);
    Serial.print(':');
    if (minute(t) < 10) Serial.print(F("0"));
    Serial.print(minute(t), DEC);
    Serial.print(':');
    if (second(t) < 10) Serial.print(F("0"));
    Serial.print(second(t), DEC);
    Serial.println();
}

void SetPinChangeInterrupt(byte Pin)  // Here is where we set the pinchange interrupt
{
    *digitalPinToPCMSK(Pin) |= bit (digitalPinToPCMSKbit(Pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(Pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(Pin)); // enable interrupt for the group
}

void ClearPinChangeInterrupt(byte Pin)  // Here is where we clear the pinchange interrupt
{
    *digitalPinToPCMSK(Pin) &= bit (digitalPinToPCMSKbit(Pin));  // disable pin
    PCIFR  != bit (digitalPinToPCICRbit(Pin)); // clear any outstanding interrupt
    PCICR  &= bit (digitalPinToPCICRbit(Pin)); // disable interrupt for the group
}

ISR (PCINT2_vect)   // interrupt service routine in sleep mode for PIR PinChange Interrupt (D0-D7)
{
    // execute code here after wake-up before returning to the loop() function
    sleep_disable ();           // first thing after waking from sleep:
    ClearPinChangeInterrupt(PIRPIN);
}

ISR (WDT_vect)      // interupt service routine for the watchdog timer
{
    // Could do something here
}  // end of WDT_vect


void sleepNow()
{
    // Here is a great tutorial on interrupts and sleep: http://www.gammon.com.au/interrupts
    Serial.print(F("Entering Sleep mode..."));
    Serial.flush ();            // wait for Serial to finish outputting
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    noInterrupts ();            // make sure we don't get interrupted before we sleep
    sleep_enable ();            // enables the sleep bit in the mcucr register
    SetPinChangeInterrupt(PIRPIN);      // Attach the PinChange Interrupt
    interrupts ();              // interrupts allowed now, next instruction WILL be executed
    sleep_cpu ();               // here the device is put to sleep
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    delay(10);                  // This small delay gives the i2c bus time to reinitialize
    Serial.println(F("Waking up"));
}

void toArduinoTime(time_t unixT) // Puts time in format for reporting
{
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    Serial.print(timeElement.Month);
    Serial.print(F("/"));
    Serial.print(timeElement.Day);
    Serial.print(F("/"));
    Serial.print(1970+timeElement.Year);
    Serial.print(F(" "));
    Serial.print(timeElement.Hour);
    Serial.print(F(":"));
    if(timeElement.Minute < 10) Serial.print(F("0"));
    Serial.print(timeElement.Minute);
    Serial.print(F(":"));
    if(timeElement.Second < 10) Serial.print(F("0"));
    Serial.print(timeElement.Second);
}

void BlinkForever() // When something goes badly wrong...
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWrite(REDLED,HIGH);
        delay(200);
        digitalWrite(REDLED,LOW);
        delay(200);
    }
}

void enable32Khz(uint8_t enable)  // Need to turn on the 32k square wave for bus moderation - could set the Talk line here
{
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission();
    
    // status register
    Wire.requestFrom(0x68, 1);
    
    uint8_t sreg = Wire.read();
    
    sreg &= ~0b00001000; // Set to 0
    if (enable == true)
        sreg |=  0b00001000; // Enable if required.
    
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(sreg);
    Wire.endTransmission();
}

int freeRam ()  // Debugging code, to check usage of RAM
{
    // Example Call: Serial.println(freeRam());
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

