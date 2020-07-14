/*
   Project: Enix Clock Firmware
   More information about this project can be found at https://www.argonprototypes.com/
   and the GitHub page https://github.com/ArgonPrototypes/enix-clock
   
   In order to compile and upload the code to the Enix using the Arduino IDE, several steps must be done:

   1. Download and install the Arduino IDE. Once open select the Arduino Micro or Genuino Micro board.
      On a Mac and PC, this can be done by going to Tools -> Board -> Arduino Micro.

   2. Install all libraries below, the ones needing to be installed have been marked. 
      This can be done on a Mac and PC by going to Sketch -> Include Library -> Manage Libraries... and searching for each
      library and clicking on the install button.

   3. Compile the code by going to the menu bar and selecting Verify/Compile. If there are any errors, let us know and maybe there is some way we can help!

   4. Press upload. The clocks digits should turn off for a couple seconds as the new program is flashed. 
   Author: Argon Prototypes
   Contributor: Carson Cook
*/

#include "Arduino.h"
#include <TimerOne.h>          /* Needs to be installed. Choose one published by Jesse Tane*/
#include <RTClib.h>            /* Needs to be installed. Choose one published by Adafruit */
#include <Adafruit_NeoPixel.h> /* Needs to be installed. Called "Adafruit NeoPixel" in Arduino Library Manager */
#include <Adafruit_MCP9808.h>  /* Needs to be installed. Called "Adafruit MCP9808 Library" in Arduino Library Manager */
#include <Wire.h>
#include <EEPROM.h>

/*
  Format: (Hardware Pin Name - Software Name)

  //PORTB = 0b(PB7)(PB6-ANODE1)(PB5-ANODE2)(PB4-ANODE3)(PB3)(PB2)(PB1)(PB0)
  //PORTC = 0b(PC7-NEOPIXEL)(PC6-COLON)(NOT USED)(NOT USED)(NOT USED)(NOT USED)(NOT USED)(NOT USED)
  //PORTD = 0b(PD7-ANODE4)(PD6-BCD2)(PD5)(PD4-BCD4)(PD3-PUSHBUTTON2)(PD2-BCD8)(PD1)(PD0)
  //PORTE = 0b(NOT USED)(PE6-PUSHBUTTON1)(NOT USED)(NOT USED)(NOT USED)(PE2)(NOT USED)(NOT USED)
  //PORTF = 0b(PF7-BCD1)(PF6)(PF5)(PF4)(NOT USED)(NOT USED)(PF1)(PF0)
*/

#define ANODE1 10 // PB6
#define ANODE2 9  // PB5
#define ANODE3 8  // PB4
#define ANODE4 6  // PD7

#define BCD1 A0 // PF7
#define BCD2 12 // PD6
#define BCD4 4  // PD4
#define BCD8 0  // PD2

#define NEOPIXEL 13 // PC7
#define COLON 5     // PC6

// 2 Push Buttons
#define PUSHBUTTON1 7 // PE6 "Menu button"
#define PUSHBUTTON2 1 // PD3 "Increment button"

// EEPROM addresses
const uint8_t BITCHECK_ADDRESS = 0; //One-time run code
const uint8_t LEDMODE_ADDRESS = 1;
const uint8_t LEDBRIGHTNESSLEVEL_ADDRESS = 2;
const uint8_t IS12HOUR_ADDRESS = 3;
const uint8_t INCELSIUS_ADDRESS = 4;
const uint8_t MODESELECTOR_ADDRESS = 5;
const uint8_t ENABLEBLINKINGCOLON_ADDRESS = 6;
const uint8_t TEMPERATUREOFFSET_ADDRESS = 7;

volatile bool isButton1ClickState = false; //Can be set by 
unsigned long lastDebounceTime1 = 0;       //keeps track of time of last button 1 click

volatile bool isButton2ClickState = false;
unsigned long lastDebounceTime2 = 0; 

volatile uint8_t currentDigit = 0; //Will cycle from 0,1,2,3 and back around every millisecond
uint16_t currentBuffer = 0000;     //Starting buffer value.

bool enableBlinkingColon = true;            //True turns on the colon, false turns it off.
volatile uint8_t enableDigits = 0b00001111; //digit1,digit2,digit3,digit4
unsigned long previousModeMillis = 0;
uint16_t hundredMillisecondCounter = 0;

volatile int8_t temperatureOffset = 0; //Manual offset of up to +/- 7 degrees Celsius
volatile bool is12hour = true;         //24 or 12 hour clock. 24 is 0, 12 is 1
volatile bool inCelsius = true;        //Fahrenheit is 0, Celsius is 1
volatile uint8_t modeSelector = 0;     //

uint8_t ledBrightnesslevel = 3; //value from 1 to 5
const uint8_t ledBrightnessMap[5] = {51, 102, 153, 204, 255};
uint8_t ledSettingsArray[][4] = {
    // special function (Rainbow), red, green, blue
    {0, 0, 0, 0},       //OFF
    {1, 0, 0, 0},       //RAINBOW
    {0, 255, 255, 255}, //WHITE
    {0, 0, 0, 255},     //BLUE
    {0, 0, 102, 255},   //LIGHT BLUE
    {0, 0, 204, 102},   //TEAL
    {0, 0, 255, 51},    //SEAFOAM GREEN
    {0, 0, 255, 0},     //GREEN
    {0, 144, 3, 252},   //LAVENDER
    {0, 255, 0, 102},   //DARK PINK
    {0, 255, 0, 0},     //RED
    {0, 255, 51, 0},    //ORANGE
    {0, 255, 170, 0}    //YELLOW

};

const uint8_t numLEDModes = sizeof(ledSettingsArray) / sizeof(ledSettingsArray[0]);
uint8_t ledModeSelect = 0;

uint8_t currentMode = 0; //Global value which holds which mode the clock is in. IE time, date, or temp
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_NeoPixel neopixels = Adafruit_NeoPixel(4, NEOPIXEL, NEO_GRB + NEO_KHZ800);
unsigned long rainbowNeoPixelsPreviousMillis = 0;
uint16_t curr8BitColor = 0;

const bool setTime = 0; //Set time using computers clock upon upload. Must be set to zero and uploaded again to continue normal operation
RTC_DS3231 rtc;

void setup()
{
  setupIO();
  setupTempSensor(); //Initialize the I2C MCP9808 temperature sensor and check everything is working
  setupRTC();        //Initialize the I2C DS3231M Real Time Clock and check everything is working
  setupTimer();
  //setupSerialPort(115200);  //Uncomment to access serial port
  setupNeoPixels();
  setupPushButtonInterrupts();

  //eraseAllEEPROM();  //Set every EEPROM cell to 0
  //setupEEPROMDefaults(); //Uncomment to set default variables

  retrieveEEPROM();
  currentBuffer = getTime(); //Load the current display buffer with the time.
}

void loop()
{
  modeHandler();
}

void setupIO()
{
  //Setup inputs and outputs. Same as pinMode(PIN, OUTPUT/INPUT);
  //A set bit (1) in a register indicates that the pin is an output.
  //Clearing a bit (0) sets the pin as an input
  DDRB |= 0b01110000; //Set outputs ANODE1, ANODE2, ANODE3 //pinMode(ANODE1, OUTPUT); pinMode(ANODE2, OUTPUT); pinMode(ANODE3, OUTPUT);
  DDRC |= 0b11000000; //Set outputs NEOPIXEL, COLON // pinMode(COLON, OUTPUT);
  DDRD |= 0b11010100; //Set outputs ANODE4, BCD2, BCD4, BCD8 // pinMode(ANODE4, OUTPUT);  pinMode(BCD2, OUTPUT); pinMode(BCD4, OUTPUT); pinMode(BCD8, OUTPUT);
  DDRD &= 0b11110111; //Clear input PUSHBUTTON2 // pinMode(PUSHBUTTON2, INPUT);
  DDRE &= 0b10111111; //Clear input PUSHBUTTON1 // pinMode(PUSHBUTTON1, INPUT);
  DDRF |= 0b10000000; //Set output BCD1 // pinMode(BCD1, OUTPUT);
}

void setupTimer()
{
  /*
     Timer1 on the ATMega32U4 is a hardware timer that can be set up to trigger an event at a precise time
     repeatedly. In this case, the multiplexing of the nixie tubes is automatically triggered regardless
     of delays in main loops. 
  */
  Timer1.initialize(1000);                   // set a timer of length 1000 microseconds / 1 millisecond
  Timer1.attachInterrupt(nixieMultiplexISR); //Every millisecond run the Interrupt Service Routine. This will display the next digit on the display, from left to right.
}

void setupSerialPort(int baudrate)
{
  Serial.begin(baudrate);
}

void setupNeoPixels()
{
  neopixels.begin(); //Initialize the neopixels
  allNeoPixelsOff(); //Turn off all neopixels to start
}

void setupPushButtonInterrupts()
{
  attachInterrupt(digitalPinToInterrupt(PUSHBUTTON1), button1ISR, FALLING); //Interrupt triggered when button 1 is pushed
  attachInterrupt(digitalPinToInterrupt(PUSHBUTTON2), button2ISR, FALLING); //Interrupt triggered when button 2 is pushed
}

//Fetch the user settings
void retrieveEEPROM()
{
  ledModeSelect = EEPROM.read(LEDMODE_ADDRESS);
  //Check for bad ledModeSelect value
  if (ledModeSelect >= numLEDModes)
    ledModeSelect = 0;

  ledBrightnesslevel = EEPROM.read(LEDBRIGHTNESSLEVEL_ADDRESS);
  if (ledBrightnesslevel < 0 || ledBrightnesslevel > 5)
    ledBrightnesslevel = 3;

  is12hour = EEPROM.read(IS12HOUR_ADDRESS);
  //Check for bad is12hour
  if (is12hour != 1 && is12hour != 0)
    is12hour = 0;

  enableBlinkingColon = EEPROM.read(ENABLEBLINKINGCOLON_ADDRESS);
  if (enableBlinkingColon != 1 && enableBlinkingColon != 0)
    enableBlinkingColon = 1;

  inCelsius = EEPROM.read(INCELSIUS_ADDRESS);
  //Check for bad inCelsius value
  if (inCelsius != 1 && inCelsius != 0)
    inCelsius = 0;

  temperatureOffset = EEPROM.read(TEMPERATUREOFFSET_ADDRESS);
  if (temperatureOffset < -7 || temperatureOffset > 7)
    temperatureOffset = 0;

  modeSelector = EEPROM.read(MODESELECTOR_ADDRESS);
  if (modeSelector < 1 || modeSelector > 4)
    modeSelector = 1;
}

//Erase the entire EEPROM. Only used when setting up a new clock.
void eraseAllEEPROM()
{
  for (int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.update(i, 0);
  }
}

//Set up EEPROM for a new clock to the default setup.
void setupEEPROMDefaults()
{
  EEPROM.update(LEDMODE_ADDRESS, 0);
  EEPROM.update(LEDBRIGHTNESSLEVEL_ADDRESS, 3);
  EEPROM.update(IS12HOUR_ADDRESS, true);
  EEPROM.update(ENABLEBLINKINGCOLON_ADDRESS, true);
  EEPROM.update(INCELSIUS_ADDRESS, true);
  EEPROM.update(TEMPERATUREOFFSET_ADDRESS, 0);
  EEPROM.update(MODESELECTOR_ADDRESS, 2);
}

/*
   Controls the normal operation of the clock. Uses polling timers to select when modes are
   shown and for how long.
   The modes that are shown can be selected in the settings menu.
   Mode 1: Time, Date, Temperature
   Mode 2: Time, Date
   Mode 3: Time, Temperature
   Mode 4: Time

   With each mode, transitions are used which cycles every digit to both be visually pleasing, and prevent cathode poisoning on the digits. 
*/

void modeHandler()
{
  if (isButton1ClickState) //Enter the settings menu if button 1 is clicked
  {
    isButton1ClickState = false;
    settingsManager();
  }
  controlLEDs();
  unsigned long currentModeMillis = millis();
  if (currentModeMillis - previousModeMillis > 100)
  { //run every hundred milliseconds
    previousModeMillis = currentModeMillis;
    hundredMillisecondCounter++;
    switch (modeSelector)
    {
    case 1: // Time, Date, Temperature
      //Global variable currentMode is used to select which mode the clock is in
      switch (currentMode)
      {
      case 0:
        handleTimeMode();
        if (hundredMillisecondCounter % 500 == 0) // 50 seconds
        {
          turnOffColon();                //turn off the colon
          nixieTransition(3, getDate()); //Transition to Date
          currentMode = 1;
          hundredMillisecondCounter = 0;
        }
        break;
      case 1:
        handleDateMode();
        if (hundredMillisecondCounter % 40 == 0) // 4 seconds
        {
          hundredMillisecondCounter = 0;
          nixieTransition(4, getTemperature());
          currentMode = 2;
        }
        break;
      case 2:
        handleTemperatureMode();
        if (hundredMillisecondCounter % 40 == 0) // 4 seconds
        {
          enableDigits = 0b1111;
          hundredMillisecondCounter = 0;
          nixieTransition(1, 0);
          currentMode = 0;
        }
        break;

      default:
        currentMode = 0;
        break;
      }

      break;

    case 2: // Time, Date
      switch (currentMode)
      {
      case 0:
        handleTimeMode();
        if (hundredMillisecondCounter % 500 == 0) // 50 seconds
        {
          turnOffColon();                //turn off the colon
          nixieTransition(3, getDate()); //Transition to Date
          currentMode = 1;
          hundredMillisecondCounter = 0;
        }
        break;
      case 1:
        handleDateMode();
        if (hundredMillisecondCounter % 40 == 0) // 4 seconds
        {
          hundredMillisecondCounter = 0;
          nixieTransition(1, 0);
          currentMode = 0;
        }
        break;
      }
      break;
    case 3: // Time, Temperature
      switch (currentMode)
      {
      case 0:
        handleTimeMode();
        if (hundredMillisecondCounter % 500 == 0) // 50 seconds
        {
          turnOffColon();                       //turn off the colon
          nixieTransition(4, getTemperature()); //Transition to Date
          currentMode = 2;
          hundredMillisecondCounter = 0;
        }
        break;

      case 2:
        handleTemperatureMode();
        if (hundredMillisecondCounter % 40 == 0) // 4 seconds
        {
          enableDigits = 0b1111;
          hundredMillisecondCounter = 0;
          nixieTransition(1, 0);
          currentMode = 0;
        }
        break;
      }
      break;
    case 4: // Time
      handleTimeMode();
      if (hundredMillisecondCounter % 500 == 0) // 50 seconds
      {
        enableDigits = 0b1111;
        hundredMillisecondCounter = 0;
        nixieTransition(1, 0);
      }
      break;
      break;
    }
  }
}

void handleTimeMode()
{
  //Only retrieve time when colon is disabled.
  if ((0b01000000 && PORTC) == 0)
  {
    currentBuffer = getTime(); //retrieve the current time
  }

  if (enableBlinkingColon && (hundredMillisecondCounter % 5 == 0))
  { //Toggle every half second
    toggleColon();
  }
}

void handleDateMode()
{
  currentBuffer = getDate();
}

void handleTemperatureMode()
{
  enableDigits = 0b0011;
  if (hundredMillisecondCounter < 2)
  { //only fetch temperature once
    currentBuffer = getTemperature();
  }
}

/*

*/
void controlLEDs()
{
  neoPixelsSetBrightness(ledBrightnesslevel);
  if (ledSettingsArray[ledModeSelect][0] == 0)
  { //not a special mode like rainbow
    for (int i = 0; i < 4; i++)
    {
      //Fetch the RGB color settings from ledSettingsArray[][]
      neopixels.setPixelColor(i, ledSettingsArray[ledModeSelect][1], ledSettingsArray[ledModeSelect][2], ledSettingsArray[ledModeSelect][3]);
    }
    neopixels.show();
  }
  else if (ledSettingsArray[ledModeSelect][0] == 1)
  {
    rainbow();
    neopixels.show();
  }
}

/*
   Manages the settings menu system.

   The settings in order are:
   1. Year    : 2019 to 2030. The purpose of the year is to decide whether it is a leap year or not
                February has 29 days instead of the usual 28. Displayed on all 4 digits with teal under lighting
   2. Month   : 1 to 12. Displayed on the left 2 digits only with green under lighting.
   3. Day     : 1 to (28,29,30,31) depending on if it is a leap year, and the current month. Displayed on the right 2 digits only with green under lighting.
   4. Hour    : 0 to 23. (0 being midnight, 23 being 11 pm.) Displayed on the left 2 digits only with blue under lighting.
   5. Minute  : 0 to 59. Displayed on the right 2 digits only with blue under lighting.

   ---- At this point the RTC is sent the new date and time and it is stored ----
   ---- Every setting after this is individually saved to EEPROM after choosing the setting ----

   6. 12/24 hour      : Either 12 or 24 is displayed on the right 2 digits with pink under lighting.
   7. Blinking Colon  : A lit neon bulb signifies blinking, unlit indicates no blinking. Teal under lighting.
   8. Under lighting  : All 4 L.E.D's light up to signify the chosen color. Order is none, rainbow, then several static colors.
   9. Under lighting brightness : 1 to 5. Displayed on the right 2 digits, with all under lighting on in the selected color, otherwise in pink to show brightness. 
   10. Temperature Units : Celsius or Fahrenheit. Displays the current temperature in either unit on the right 2 digits, with red under lighting.
   11. Temperature Offset : -7 to 7 in degrees Celsius (Each degree C is 1.8 degrees F). Displayed on the right 2 digits with pink under lighting. The center neon colon acts as a negative sign.
   12. Mode Select    : 1 to 4. Shown on the right 2 digits with blue underlighting.
                          Mode 1: Time, Date, Temperature
                          Mode 2: Time, Date
                          Mode 3: Time, Temperature
                          Mode 4: Time 
    ---- Exit the menu system ----
*/

void settingsManager()
{
  isButton2ClickState = false; //reset global flag
  bool copyOfis12hour = is12hour;
  bool copyOfBlinkingColon = enableBlinkingColon;
  uint8_t copyOfModeSelector = modeSelector;
  is12hour = false; //Make 24 hour clock for setting
  uint16_t currentDate = getDate();
  uint16_t currentTime = getTime();
  uint8_t currentMonth = currentDate / 100;
  uint8_t currentDay = currentDate % 100;
  uint8_t currentHour = currentTime / 100;
  uint8_t currentMinute = currentTime % 100;
  uint16_t currentYear = getYear();
  turnOffColon();
  currentBuffer = currentHour;
  allNeoPixelsOff();

  //Setting 1: Choose Year
  currentYear = chooseYear(currentYear);
  //Setting 2: Choose month
  currentMonth = chooseMonth(currentMonth);
  //Setting 3: Choose Day
  currentDay = chooseDay(currentDay, currentYear, currentMonth);
  //Setting 4: Choose hour
  currentHour = chooseHour(currentHour);
  //Setting 5: Choose minute
  currentMinute = chooseMinute(currentMinute);
  //Send the new time to the RTC
  rtc.adjust(DateTime(currentYear, currentMonth, currentDay, currentHour, currentMinute, 0));

  //Choose 24 or 12 time setting
  is12hour = chooseis12hour(copyOfis12hour);
  EEPROM.update(IS12HOUR_ADDRESS, is12hour);

  //Choose Blinking Colon
  enableBlinkingColon = chooseEnableBlinkingColon(copyOfBlinkingColon);
  EEPROM.update(ENABLEBLINKINGCOLON_ADDRESS, enableBlinkingColon);

  //Choose LED underlighting settings
  ledModeSelect = chooseLEDMode();
  EEPROM.update(LEDMODE_ADDRESS, ledModeSelect);

  //Choose LED brightness settings
  chooseLEDBrightness();
  EEPROM.update(LEDBRIGHTNESSLEVEL_ADDRESS, ledBrightnesslevel);

  //Choose Fahrenheit/Celsius setting
  inCelsius = chooseInCelsius();
  EEPROM.update(INCELSIUS_ADDRESS, inCelsius);

  temperatureOffset = chooseTemperatureOffset();
  EEPROM.update(TEMPERATUREOFFSET_ADDRESS, temperatureOffset);

  modeSelector = chooseModeSelector(modeSelector);
  EEPROM.update(MODESELECTOR_ADDRESS, modeSelector);

  //Restore
  isButton1ClickState = false;
  enableDigits = 0b1111;
  currentMode = 0;
  allNeoPixelsOff();
}

uint8_t chooseHour(uint8_t currentHour)
{
  isButton1ClickState = false;
  enableDigits = 0b1100;                 // turn off right 2 digits
  neopixels.setPixelColor(3, 0, 0, 255); //underlight left most LED
  neopixels.setPixelColor(2, 0, 0, 255); //underlight one over from the left most
  neopixels.show();

  while (!isButton1ClickState)
  {
    currentBuffer = currentHour * 100; // shift hour val to the left 2 digits
    if (isButton2ClickState)
    {                              // if button1
      isButton2ClickState = false; //reset button 2 counter
      if (currentHour == 23)
      {
        currentHour = 0; //rollover
      }
      else
      {
        currentHour++;
      }
    }
    Serial.print("Hour: ");
    Serial.println(currentHour);
    delay(100);
  }
  allNeoPixelsOff(); // turn off all pixels
  return currentHour;
}

uint8_t chooseMinute(uint8_t currentMinute)
{
  isButton1ClickState = false; //reset isButton1ClickState
  neopixels.setPixelColor(1, 0, 0, 255);
  neopixels.setPixelColor(0, 0, 0, 255);
  neopixels.show();
  enableDigits = 0b0011;
  while (!isButton1ClickState)
  {
    currentBuffer = currentMinute; //currentMinute;
    if (isButton2ClickState)
    {                              //button 2 incremented
      isButton2ClickState = false; //reset global flag
      if (currentMinute == 59)
      {
        currentMinute = 0;
      }
      else
      {
        currentMinute++;
      }
    }
    Serial.print("Minute: ");
    Serial.println(currentMinute);
    delay(100);
  }
  allNeoPixelsOff();
  return currentMinute;
}

uint16_t chooseYear(uint16_t currentYear)
{
  isButton1ClickState = false; //reset isButton1ClickState
  neopixels.setPixelColor(0, 0, 255, 128);
  neopixels.setPixelColor(1, 0, 255, 128);
  neopixels.setPixelColor(2, 0, 255, 128);
  neopixels.setPixelColor(3, 0, 255, 128);
  neopixels.show();

  if (currentYear < 2018 || currentYear > 2030)
    currentYear = 2018;
  currentBuffer = currentYear;
  enableDigits = 0b1111;
  while (!isButton1ClickState)
  {
    currentBuffer = currentYear;
    if (isButton2ClickState)
    { //button 2 incremented
      isButton2ClickState = false;
      if (currentYear >= 2030)
      {
        currentYear = 2018;
      }
      else
        currentYear++;
    }
    delay(100);
  }
  allNeoPixelsOff();
  return currentYear;
}

uint8_t chooseMonth(uint8_t currentMonth)
{
  isButton1ClickState = false; //reset isButton1ClickState
  enableDigits = 0b1100;
  neopixels.setPixelColor(2, 0, 255, 0);
  neopixels.setPixelColor(3, 0, 255, 0);
  neopixels.show();
  while (!isButton1ClickState)
  {
    currentBuffer = currentMonth * 100;
    if (isButton2ClickState)
    {                              //button 2 incremented
      isButton2ClickState = false; //reset global flag
      if (currentMonth == 12)
      {
        currentMonth = 1;
      }
      else
      {
        currentMonth++;
      }
    }
    delay(100);
  }
  allNeoPixelsOff();
  return currentMonth;
}

uint8_t chooseDay(uint8_t day, uint16_t year, uint8_t month)
{
  isButton1ClickState = false; //reset isButton1ClickState
  enableDigits = 0b1100;
  neopixels.setPixelColor(1, 0, 255, 0);
  neopixels.setPixelColor(0, 0, 255, 0);
  neopixels.show();
  enableDigits = 0b0011;
  if (day > daysInMonth(month, year))
  {
    day = 1;
  }
  while (!isButton1ClickState)
  {
    currentBuffer = day;
    if (isButton2ClickState)
    {                              //button 2 incremented
      isButton2ClickState = false; //reset global flag

      if (day >= daysInMonth(month, year))
      {
        day = 1; // rollover day in the month select
      }
      else
      {
        day++;
      }
    }
    delay(100);
  }
  allNeoPixelsOff();
  return day;
}

uint8_t chooseLEDMode()
{
  enableDigits = 0b0000;                                                               //turn off all digits
  isButton1ClickState = false;                                                         //reset isButton1ClickState
  uint8_t numRowsLEDSettings = sizeof(ledSettingsArray) / sizeof(ledSettingsArray[0]); //number of led options
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {                              //button 2 incremented
      isButton2ClickState = false; //reset global flag
      if (ledModeSelect == numRowsLEDSettings - 1)
      {
        ledModeSelect = 0;
      }
      else
      {
        ledModeSelect++;
      }
    }
    controlLEDs();
    delay(5);
  }
  allNeoPixelsOff();
  return ledModeSelect;
}

void chooseLEDBrightness()
{
  enableDigits = 0b0011;
  currentBuffer = ledBrightnesslevel;
  uint8_t copyOfLEDModeSelect = ledModeSelect;
  if (ledModeSelect == 0 || ledModeSelect == 1)
  {
    ledModeSelect = 3;
  }
  controlLEDs();
  isButton1ClickState = false; //reset isButton1ClickState
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {                              //button 2 incremented
      isButton2ClickState = false; //reset global flag
      if (ledBrightnesslevel >= 5)
      {
        ledBrightnesslevel = 1;
      }
      else
      {
        ledBrightnesslevel++;
      }
      currentBuffer = ledBrightnesslevel;
      neoPixelsSetBrightness(ledBrightnesslevel);
      neopixels.show();
    }
    delay(5);
  }
  ledModeSelect = copyOfLEDModeSelect; //restore the previously chosen LED mode
  allNeoPixelsOff();
}

bool chooseis12hour(bool copyOfis12hour)
{
  isButton1ClickState = false; //reset isButton1ClickState
  enableDigits = 0b0011;       //turn on right 2 digits
  is12hour = copyOfis12hour;   //restore is12hour
  neopixels.setPixelColor(1, 255, 0, 255);
  neopixels.setPixelColor(0, 255, 0, 255);
  neopixels.show();
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {
      isButton2ClickState = false;
      is12hour = !is12hour; //toggle from 12/24
    }
    if (is12hour)
      currentBuffer = 12;
    else
      currentBuffer = 24;
    delay(100);
  }
  allNeoPixelsOff();
  return is12hour;
}

bool chooseEnableBlinkingColon(bool enableBlinkingColon)
{
  isButton1ClickState = false; //reset isButton1ClickState
  enableDigits = 0b0000;
  uint8_t copyOfLedBrightnesslevel = ledBrightnesslevel;
  ledBrightnesslevel = 3;
  neoPixelsSetBrightness(ledBrightnesslevel);
  neopixels.setPixelColor(1, 0, 100, 100);
  neopixels.setPixelColor(2, 0, 100, 100);
  neopixels.show();
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {
      isButton2ClickState = false;
      enableBlinkingColon = !enableBlinkingColon; //toggle colon on/off
    }
    if (enableBlinkingColon)
      turnOnColon();
    else
      turnOffColon();
    delay(100);
  }
  allNeoPixelsOff();
  ledBrightnesslevel = copyOfLedBrightnesslevel;
  neoPixelsSetBrightness(ledBrightnesslevel);
  turnOffColon();
  return enableBlinkingColon;
}

bool chooseInCelsius()
{
  isButton1ClickState = false; //reset isButton1ClickState
  enableDigits = 0b0011;       //turn on right 2 digits
  neopixels.setPixelColor(1, 255, 0, 0);
  neopixels.setPixelColor(0, 255, 0, 0);
  neopixels.show();
  currentBuffer = getTemperature();
  uint8_t getTemperatureCounter = 0;
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {
      isButton2ClickState = false;
      inCelsius = !inCelsius; //toggle from Celsius/Fahrenheit
      currentBuffer = getTemperature();
      getTemperatureCounter = 0;
    }
    if (getTemperatureCounter == 100) //fetch temperature every so often
    {
      currentBuffer = getTemperature();
      getTemperatureCounter = 0;
    }
    getTemperatureCounter++;

    delay(100);
  }
  allNeoPixelsOff();
  return inCelsius;
}

int8_t chooseTemperatureOffset()
{
  isButton1ClickState = false; //reset isButton1ClickState
  neopixels.setPixelColor(1, 227, 3, 252);
  neopixels.setPixelColor(0, 227, 3, 252);
  neopixels.show();
  enableDigits = 0b0011; //turn on right 2 digits
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {
      isButton2ClickState = false;
      if (temperatureOffset >= 7)
        temperatureOffset = -7;
      else
        temperatureOffset += 1;
    }
    Serial.print("Temperature Offset: ");
    Serial.println(temperatureOffset);
    uint8_t positiveTemperatureOffset;
    if (temperatureOffset < 0)
    {
      positiveTemperatureOffset = abs(temperatureOffset);
      Serial.print("Temperature Offset Absolute: ");
      Serial.println(positiveTemperatureOffset);
      turnOnColon();
      currentBuffer = positiveTemperatureOffset;
    }
    else
    {
      turnOffColon();
      currentBuffer = temperatureOffset;
    }
    delay(100);
  }
  allNeoPixelsOff();
  turnOffColon();
  return temperatureOffset;
}

uint8_t chooseModeSelector(uint8_t modeSelector)
{
  isButton1ClickState = false; //reset isButton1ClickState
  enableDigits = 0b0011;       //turn on right 2 digits
  neopixels.setPixelColor(0, 30, 100, 100);
  neopixels.setPixelColor(1, 30, 100, 100);
  neopixels.show();
  while (!isButton1ClickState)
  {
    if (isButton2ClickState)
    {
      isButton2ClickState = false;
      if (modeSelector >= 4)
        modeSelector = 1;
      else
        modeSelector++;
    }
    currentBuffer = modeSelector;
    delay(100);
  }
  allNeoPixelsOff();
  return modeSelector;
}
/*
   Anti-Cathode poisoning transitions.

*/
void nixieTransition(uint8_t type, uint16_t nextNumber)
{

  if (isBadFourDigitNumber(nextNumber))
  {
    handleBadNumber(nextNumber);
  }
  else
  {
    handleTransition(type, nextNumber);
  }
}

void handleTransition(uint8_t type, uint16_t nextNumber)
{

  switch (type)
  {
  case 0:
    scrollDigits0To9();
    break;

  case 1:
    scrollDigitsBackAndForth();
    break;

  //Effect 2: Digit 3,2,1,0, cycling through 0-9 until they are set in order
  case 2:
    cascadingSettingOfDigits(nextNumber);
    break;

  //Effect 3: All digits randomly scroll and randomly digits are set one at a time until all are set
  case 3:
    randomlySetupDigits(nextNumber);
    break; //end case 3

  //Transition from all 4 digits to only the most right 2 digits on.
  //Used to transition from date to temperature
  case 4:
    transitionDateToTemperature(nextNumber);
    break;

  //Basic transition
  default:
    scrollDigits3Times();
  }
}

void scrollDigits0To9()
{
  for (uint8_t i = 0; i < 6; i++)
  {
    for (uint16_t j = 0; j <= 9999; j += 1111)
    {
      currentBuffer = j;
      delay(25);
    }
  }
}

void scrollDigitsBackAndForth()
{
  uint16_t orderOfDigits[10] = {1111, 0000, 2222, 9999, 3333, 8888, 4444, 7777, 5555, 6666};

  for (uint8_t i = 0; i < 2; i++)
  {
    for (uint8_t j = 0; j <= 9; j++)
    {
      currentBuffer = orderOfDigits[j];
      delay(25);
    }
    for (uint8_t k = 8; k > 1; k--)
    {
      currentBuffer = orderOfDigits[k];
      delay(25);
    }
  }
}

/*
Sets digits from 
*/
void cascadingSettingOfDigits(uint16_t nextNumber)
{
  uint8_t arrCurrentBuffer[4] = {(currentBuffer / 1000) % 10, (currentBuffer / 100) % 10, (currentBuffer / 10) % 10, currentBuffer % 10};
  uint8_t arrNextBuffer[4] = {((int)(nextNumber / 1000) % 10), (nextNumber / 100) % 10, (nextNumber / 10) % 10, nextNumber % 10};
  for (int digit = 3; digit >= 0; digit--)
  { //start at the most right digit and progress to the left
    for (uint8_t i = 0; i <= 9; i++)
    {
      arrCurrentBuffer[digit] = i;
      currentBuffer = (arrCurrentBuffer[0] * 1000) + (arrCurrentBuffer[1] * 100) + (arrCurrentBuffer[2] * 10) + (arrCurrentBuffer[3]);
      delay(20);
    }
    arrCurrentBuffer[digit] = arrNextBuffer[digit];
    currentBuffer = arrCurrentBuffer[0] * 1000 + arrCurrentBuffer[1] * 100 + arrCurrentBuffer[2] * 10 + arrCurrentBuffer[3];
  }
}

void randomlySetupDigits(uint16_t nextNumber)
{
  uint8_t orderOfSetDigitsArr[4] = {0, 1, 2, 3}; //Set up an array to be later randomly shuffled around
  uint8_t arrCurrentBuffer[4] = {(currentBuffer / 1000) % 10, (currentBuffer / 100) % 10, (currentBuffer / 10) % 10, currentBuffer % 10};
  uint8_t arrNextBuffer[4] = {((int)(nextNumber / 1000) % 10), (nextNumber / 100) % 10, (nextNumber / 10) % 10, nextNumber % 10};
  uint8_t digitsToScramble = 0b00001111; //Keep track of which digits have been set. E.G. 0b00001010 has digit 1 and 3 set.

  for (uint8_t i = 0; i <= 20; i++)
  {                                              //Swap around the order of
    uint8_t randomNum1 = genRandomDigitToSwap(); //random number from 0-3
    uint8_t randomNum2 = genRandomDigitToSwap();
    uint8_t val1 = orderOfSetDigitsArr[randomNum1];
    orderOfSetDigitsArr[randomNum1] = orderOfSetDigitsArr[randomNum2];
    orderOfSetDigitsArr[randomNum2] = val1;
  }

  for (int digit = 0; digit <= 3; digit++)
  { //start at the most right digit and progress to the left
    for (uint8_t i = 0; i <= 9; i++)
    {
      if (digitsToScramble & 1)
      { //if the digit has not been set yet
        arrCurrentBuffer[0] = i;
      }
      if (digitsToScramble & (1 << 1))
      {
        arrCurrentBuffer[1] = i;
      }
      if (digitsToScramble & (1 << 2))
      {
        arrCurrentBuffer[2] = i;
      }
      if (digitsToScramble & (1 << 3))
      {
        arrCurrentBuffer[3] = i;
      }
      currentBuffer = arrCurrentBuffer[0] * 1000 + arrCurrentBuffer[1] * 100 + arrCurrentBuffer[2] * 10 + arrCurrentBuffer[3];
      delay(20);
    }
    arrCurrentBuffer[orderOfSetDigitsArr[digit]] = arrNextBuffer[orderOfSetDigitsArr[digit]];
    digitsToScramble &= ~(1 << orderOfSetDigitsArr[digit]);
    currentBuffer = arrCurrentBuffer[0] * 1000 + arrCurrentBuffer[1] * 100 + arrCurrentBuffer[2] * 10 + arrCurrentBuffer[3];
  }
}

void transitionDateToTemperature(uint16_t nextNumber)
{
  uint8_t arrCurrentBuffer[4] = {(currentBuffer / 1000) % 10, (currentBuffer / 100) % 10, (currentBuffer / 10) % 10, currentBuffer % 10};
  uint8_t arrNextBuffer[4] = {((int)(nextNumber / 1000) % 10), (nextNumber / 100) % 10, (nextNumber / 10) % 10, nextNumber % 10};
  uint8_t digit;
  for (digit = 0; digit <= 3; digit++)
  { //start at the most left digit, and move right.
    for (uint8_t i = 0; i <= 9; i++)
    {
      arrCurrentBuffer[digit] = i;
      currentBuffer = (arrCurrentBuffer[0] * 1000) + (arrCurrentBuffer[1] * 100) + (arrCurrentBuffer[2] * 10) + (arrCurrentBuffer[3]);
      delay(20);
    }
    arrCurrentBuffer[digit] = arrNextBuffer[digit];
    currentBuffer = arrCurrentBuffer[0] * 1000 + arrCurrentBuffer[1] * 100 + arrCurrentBuffer[2] * 10 + arrCurrentBuffer[3];
    switch (digit)
    {
    case 0:
      enableDigits = 0b00000111;
      break;
    case 1:
      enableDigits = 0b00000011;
      break;
    default:
      break;
    }
  }
}
uint8_t genRandomDigitToSwap()
{
  return rand() % 4;
}

void scrollDigits3Times()
{
  uint16_t orderOfDigits[10] = {1029, 293, 2938, 9384, 3847, 8475, 4756, 7561, 5610, 6102};
  for (uint8_t i = 0; i < 3; i++)
  {
    for (uint8_t j = 0; j <= 9; j++)
    {
      currentBuffer = orderOfDigits[j];
      delay(25);
    }
  }
}

void handleBadNumber(uint16_t nextNumber)
{
  Serial.print("Error, nextNumber is incorrect: ");
  Serial.println(nextNumber);
}

/*

*/
void button1ISR()
{
  unsigned long button1DebounceDelay = 400; //millisecond debounce to recognize another button press
  if (isDelayToRecognizePress(button1DebounceDelay + lastDebounceTime1))
  {
    lastDebounceTime1 = millis();
    isButton1ClickState = true;
  }
}

/*

*/
void button2ISR()
{
  unsigned long button2DebounceDelay = 300;
  if (isDelayToRecognizePress(button2DebounceDelay + lastDebounceTime2))
  {
    lastDebounceTime2 = millis();
    isButton2ClickState = true;
  }
}

bool isDelayToRecognizePress(unsigned long delay)
{
  return millis() > delay;
}

void setupTempSensor()
{
  checkTempSensorBegan();
}

void checkTempSensorBegan()
{
  if (!tempsensor.begin())
  {
    Serial.println("Couldn't find MCP9808!");
  }
}

/*

*/
uint16_t getYear()
{
  uint8_t copyOfPORTC = PORTC;
  turnOffColon();
  DateTime now = rtc.now(); //DateTime object returned into now
  PORTC = copyOfPORTC;
  return now.year();
}

/*
   Input a given year to check if it is a leap year.
   During a leap year, the month of February (2) will have
   29 days rather than 28. This occurs roughly every 4 years.
   Return true if given year is a leap year, false if not
*/
bool isLeapYear(uint16_t year)
{
  uint16_t leapYears[21] = {2020, 2024, 2028, 2032, 2036, 2040, 2044, 2048, 2052, 2056, 2060, 2064, 2068, 2072, 2076, 2080, 2084, 2088, 2092, 2096, 2104};

  for (uint8_t i = 0; i < sizeof(leapYears) / sizeof(leapYears[0]); i++)
  {
    if (year == leapYears[i])
    {
      return true;
    }
  }
  return false;
}

bool isBadFourDigitNumber(uint16_t nextNumber)
{
  return nextNumber > 9999 || nextNumber < 0;
}

/*
Used for setting time in the menu system. The RTC has its own
settings for leap years, however this is used to prevent setting
an impossible date
*/
uint8_t daysInMonth(uint8_t month, uint16_t year)
{

  uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //January, February, ... , December
  if (isLeapYear(year))                                                //If it is a leap year
    days[1] = 29;                                                      //29 days instead of 28.
  if (month > 0 && month < 13)
    return days[month - 1];
}

/*
   Returns 4 digits representing the month and day in format MMDD
*/
uint16_t getDate()
{
  uint8_t copyOfPORTC = PORTC;
  turnOffColon();
  DateTime now = rtc.now(); //DateTime object returned into now
  PORTC = copyOfPORTC;
  return formatDate(now);
}

uint16_t formatDate(DateTime now)
{
  return now.month() * 100 + now.day();
}

/*
   Returns 4 digits in format HHMM in 24hr format if is12hour == 1
*/
uint16_t getTime()
{
  uint8_t copyOfPORTC = PORTC;
  turnOffColon();
  DateTime now = rtc.now(); //DateTime object returned into now
  PORTC = copyOfPORTC;      //After reading I2C bus, return PORTC.
  printTimeDate(now);
  return formatTimeStamp(now);
}

uint16_t formatTimeStamp(DateTime now)
{

  uint16_t currentTime;
  if (is12hour)
  {                      //time format = 1, 12 hour
    if (isMidnight(now)) //if midnight, show 12AM
      currentTime = convertToMidnight(now);
    else if (now.hour() > 12)
      currentTime = convertTo12HourTimeStamp(now);
    else
      currentTime = convertTo24HourTimestamp(now);
  }
  else
  { //24 hour time
    currentTime = convertTo24HourTimestamp(now);
  }
  return currentTime;
}

bool isMidnight(DateTime now)
{
  return now.hour() == 0;
}

uint16_t convertToMidnight(DateTime now)
{
  return (12) * 100 + now.minute();
}

uint16_t convertTo12HourTimeStamp(DateTime now)
{
  return (now.hour() - 12) * 100 + now.minute();
}

uint16_t convertTo24HourTimestamp(DateTime now)
{
  return now.hour() * 100 + now.minute();
}

void printTimeDate(DateTime now)
{
  //Serial Prints for debugging time and date.
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
}

/*


*/
void setupRTC()
{
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  if (setTime)
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

/*
   Function which retrieves temperature from the I2C bus MCP9808 temperature sensor
   and returns a 8 bit positive integer in Celsius or Fahrenheit.
   if global bool inCelsius is 1, it will return Celsius,
   0 will return Fahrenheit
   Due to an increased circuit board operating temperature, there is an adjustment
   applied to the value based on how long the circuit has been powered on for.
*/
uint8_t getTemperature()
{
  unsigned long adjustTemp; //adjust temperature
  unsigned long millisTime = millis();
  if (millisTime > 3000000)
    millisTime = 3000000;
  // Adjust temperature from 0 degrees at 0 minutes to -7 degrees at 3,000,000 milliseconds or 50 minutes to
  // compensate for PCB rise in temperature
  adjustTemp = (millisTime - 0.0) * (7.0 - 0.0) / (3000000.0 - 0.0) + 0.0; //map(millisTime, 0, 1000000, 0, 8.0);
  Serial.print("Adjust Temperature Value: ");
  Serial.println(adjustTemp);
  Serial.print("Set Temperature Offset: ");
  Serial.println(temperatureOffset);
  //Disable the neon colon bulb prior to reading I2C temperature
  uint8_t copyOfPORTC = PORTC;
  turnOffColon();
  float temp;
  for (int i = 0; i < 3; i++)
  { //Get an average of 4 readings
    temp += tempsensor.readTempC() - adjustTemp + temperatureOffset;
  }
  temp = temp / 3;
  if (temp < 0)
    temp = 0;

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.println("*C\t");
  if (!inCelsius)
  {
    temp = convertToFahrenheit(temp);
    if (temp > 99)
      temp = 99;
  }
  temp = floor(temp);
  uint8_t tempInt = (int)temp;
  PORTC = copyOfPORTC;
  return tempInt;
}

float convertToFahrenheit(float temp)
{
  return temp * 9.0 / 5.0 + 32;
}

/*
  Rainbow LED Mode  
*/
void rainbow()
{
  unsigned long currentMillis = millis(); //get a copy of current millis() value
  if (currentMillis - rainbowNeoPixelsPreviousMillis >= 60)
  {
    rainbowNeoPixelsPreviousMillis = currentMillis;
    setRainbowColor(curr8BitColor);
    curr8BitColor++;
    neopixels.show();
  }
}

void setRainbowColor(uint8_t color)
{
  color = cycleColorIfNeed(color);
  for (uint8_t currPixel = 0; currPixel < 4; currPixel++)
  {
    neopixels.setPixelColor(currPixel, Wheel((currPixel + color) & 255));
  }
  neopixels.show();
}

uint8_t cycleColorIfNeed(uint8_t curr8BitColor)
{
  if (isBadColor(curr8BitColor))
  {
    return 0;
  }
  return curr8BitColor;
}

bool isBadColor(uint8_t color)
{
  return color >= 255;
}

void neoPixelIndividualOff(uint8_t pixelNum)
{ //0 to 3, 0 being the most right led.
  neopixels.setPixelColor(pixelNum, 0, 0, 0);
}

void neoPixelsStaticColor(uint8_t red, uint8_t green, uint8_t blue)
{
  for (uint8_t i = 0; i < neopixels.numPixels(); i++)
  {
    neopixels.setPixelColor(i, red, green, blue);
  }
  neopixels.show();
}

//Brightness level from 1-5
void neoPixelsSetBrightness(uint8_t brightness)
{
  if (brightness > 5 || brightness < 1)
    brightness = 5;
  brightness = ledBrightnessMap[brightness - 1];
  neopixels.setBrightness(brightness);
}

void allNeoPixelsOff()
{
  clearPixels();
  neopixels.show();
}

void clearPixels()
{
  for (uint8_t i = 0; i < neopixels.numPixels(); i++)
  {
    neoPixelIndividualOff(i);
  }
}

uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return neopixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return neopixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return neopixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void toggleColon()
{
  PORTC ^= 0b01000000; //toggle the colon on and off
}

void turnOnColon()
{
  PORTC |= 0b01000000;
}

void turnOffColon()
{
  PORTC &= 0b10111111;
}

void nixieMultiplexISR()
{
  cycleCurrentDigit();
  clearAnodes();
  uint8_t currentNumber = setAnodeForCurDigit();
  clearBCD();
  setBCD(currentNumber);
}

void cycleCurrentDigit()
{
  //Digit 0 is far left, and 3 is the far right digit
  if (currentDigit >= 3)
  {
    currentDigit = 0;
  }
  else
  {
    currentDigit++;
  }
}

void clearAnodes()
{
  //Clear Anodes
  PORTB &= 0b10001111; //Clear Anode (1,2,3) aka (PB6,PB5,PB4) aka (D10, D9, D8)
  PORTD &= 0b01111111; //Clear Anode (4) aka (PD7) aka D6
}

uint8_t setAnodeForCurDigit()
{

  uint8_t currentNumber;
  //Set Anode for current digit
  if (enableDigits == 0b0011)
  {
    currentNumber = setAnodeRight2Digits();
  }
  else if (enableDigits == 0b0111)
  {
    currentNumber = setAnodeRight3Digits();
  }
  else if (enableDigits == 0b1100)
  {
    currentNumber = setAnodeLeft2Digits();
  }
  else
  {
    switch (currentDigit)
    {
    case 0: //Set PB6
      if (shouldAnodeBeOn(enableDigits, 3))
      { //check if anode should be turned on
        currentNumber = (int)(currentBuffer / 100 / 10) % 10;
        PORTB |= 0b01000000; // Set Anode 1 PB6
      }
      break;

    case 1: //Set PB5
      if (shouldAnodeBeOn(enableDigits, 2))
      {
        PORTB |= 0b00100000;
        currentNumber = (currentBuffer / 100) % 10;
      }
      break;

    case 2:
      //Set Anodes for tens digit
      if (shouldAnodeBeOn(enableDigits, 1))
      {
        PORTB |= 0b00010000;
        currentNumber = (currentBuffer / 10) % 10;
        break;
      }

    case 3:
      if (shouldAnodeBeOn(enableDigits, 0))
      {
        PORTD |= 0b10000000;
        currentNumber = currentBuffer % 10;
      }
      break;
    }
  }
  return currentNumber;
}

bool shouldAnodeBeOn(uint8_t enableDigits, int digit)
{
  return (1 << digit) & enableDigits;
}

/*
 Multiplexing schedule for when the left 2 digits are turned off, which is
 used for the temperature mode. 
 Rather than waste 2 cycles on the unlit digits, re-allocate the multiplexing
 for the right two digits. The benefit of this is less electric noise from the SMPS from uneven
 load. The disadvantage is that under this mode, the lit digits are 
 slightly brighter than when all 4 are on. 
 */
uint8_t setAnodeRight2Digits()
{
  uint8_t currentNumber;
  switch (currentDigit)
  {
  case 0: //Set PB6
    PORTB |= 0b00010000;
    currentNumber = (currentBuffer / 10) % 10;
    break;

  case 1: //Set PB5

    PORTD |= 0b10000000;
    currentNumber = currentBuffer % 10;
    break;

  case 2:
    //Set Anodes for tens digit
    PORTB |= 0b00010000;
    currentNumber = (currentBuffer / 10) % 10;
    break;

  case 3:
    PORTD |= 0b10000000;
    currentNumber = currentBuffer % 10;
    break;
  }
  return currentNumber;
}

/*
  Similar to setAnodeForTemperatureMode(), but for when the 
  the left single digit is unlit.
 */
uint8_t setAnodeRight3Digits()
{
  uint8_t currentNumber;
  switch (currentDigit)
  {
  case 0: //Set PB6
    PORTB |= 0b00100000;
    currentNumber = (currentBuffer / 100) % 10;
    break;

  case 1: //Set PB5
    PORTB |= 0b00100000;
    currentNumber = (currentBuffer / 100) % 10;
    break;

  case 2:
    PORTB |= 0b00010000;
    currentNumber = (currentBuffer / 10) % 10;
    break;

  case 3:
    PORTD |= 0b10000000;
    currentNumber = currentBuffer % 10;
    break;
  }
  return currentNumber;
}

uint8_t setAnodeLeft2Digits()
{
  uint8_t currentNumber;
  switch (currentDigit)
  {
  case 0: //Set PB6
    currentNumber = (int)(currentBuffer / 100 / 10) % 10;
    PORTB |= 0b01000000; // Set Anode 1 PB6
    break;

  case 1:
    currentNumber = (currentBuffer / 100) % 10;
    PORTB |= 0b00100000; // Set Anode 2 PB5
    break;

  case 2:
    currentNumber = (int)(currentBuffer / 100 / 10) % 10;
    PORTB |= 0b01000000; // Set Anode 1 PB6
    break;

  case 3:
    currentNumber = (currentBuffer / 100) % 10;
    PORTB |= 0b00100000; // Set Anode 2 PB5
    break;
  }
  return currentNumber;
}
void clearBCD()
{
  //Clear BCD
  PORTD &= 0b10101011; //Clear BCD (2,4,8) aka (PD6,PD4,PD2) aka (D12,D4,D0)
  PORTF &= 0b01111111; //Clear BCD (1) aka (PF7) aka (A0)
}

void setBCD(uint8_t currentNumber)
{
  //Switch case from 0-9
  switch (currentNumber)
  {
  //As reference BCD: 1248, PF7,PD6,PD4,PD2
  case 0: //BCD: 1110: Set PF7, PD6, and PD4
    PORTF |= 0b10000000;
    PORTD |= 0b01010000;
    break;

  case 1: //BCD: 1100: Set PF7 and PD6
    PORTF |= 0b10000000;
    PORTD |= 0b01000000;
    break;

  case 2: //BCD: 0100: Set PD6
    PORTD |= 0b01000000;
    break;

  case 3: //BCD: 1001: Set PF7 and PD2
    PORTF |= 0b10000000;
    PORTD |= 0b00000100;
    break;

  case 4: //BCD: 0001: Set PD2
    PORTD |= 0b00000100;
    break;

  case 5: //BCD: 0000: Don't set anything
    //Leave outputs as 0's
    break;

  case 6: //BCD: 1000: Set PF7
    PORTF |= 0b10000000;
    break;

  case 7: //BCD: 1010: Set PF7 and PD4
    PORTF |= 0b10000000;
    PORTD |= 0b00010000;
    break;

  case 8: //BCD: 0010: Set PD4
    PORTD |= 0b00010000;
    break;

  case 9: //BCD: 0110: Set PD6 and PD4
    PORTD |= 0b01010000;
    break;
  }
}

