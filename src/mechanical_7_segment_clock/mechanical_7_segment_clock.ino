#include "SPI.h"
#include "Wire.h"
#include "RTClib.h"
uint8_t segmentOrder[12] = {0b01111110, 0b00110000, 0b01101101, 0b01111001, 0b00110011, 0b01011011, 0b01011111, 0b01110000, 0b01111111, 0b01111011, 0b00000001, 0b00000000}; //
// ####### Segement order numbers #######
// _       _   _       _   _   _   _   _
//| |   |  _|  _| |_| |_  |_    | |_| |_|  _
//|_|   | |_   _|   |  _| |_|   | |_|  _|
// 0    1   2   3   4   5   6   7   8   9  10   11   // 11 is all segments in off position
const uint8_t stepOrder[9] = {0b00000000, 0b00000001, 0b00000011, 0b00000010, 0b00000110, 0b00000100, 0b00001100, 0b00001000, 0b00001001}; //binary order for stepper movement
const int8_t boardMap[28] = {2, 9, 3, 10, 4, 11, 0, 7, 5, 12, 6, 13, 1, 8, 26, 19, 27, 20, 22, 15, 21, 14, 23, 16, 24, 17, 25, 18};        //mapping of the board pin output to segments
const int8_t direction[4] = {1, -1, -1, 1};                                                                                                // Direction to move to open all segments
int8_t currentStep[28] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint32_t stepsPerMove = 1200; //1200;  /// set correctly 4096/rev
uint32_t stepRate = 900;      //delay between steps in micro seconds min 800
uint32_t mask;
uint8_t byteOut = 0b00000000;
uint8_t nibbleOutCount = 0;
const int slaveSelectPin = 10;
uint32_t timer;
const char compile_date[] = __DATE__ " " __TIME__;
byte Minutes = 0;
byte Hour = 0;
uint8_t timeArray[4] = {11, 11, 11, 11}; //{min digit, min tens,hour digit, hour tens} // 11 is a fully open digit IE all segments folded back
uint8_t oldTimeArray[4] = {8, 8, 8, 8};  //{min digit, min tens,hour digit, hour tens}
bool offOn = true;
unsigned long sleepTimerNew = 0;
unsigned long sleepTimerOld = 0;

RTC_DS3231 rtc;
#define CLOCK_INTERRUPT_PIN 2

uint8_t buttonPin[4] = {6, 7, 5, 4}; //
uint8_t ButtonState[4] = {LOW, LOW, LOW, LOW};
uint8_t lastButtonState[4] = {LOW, LOW, LOW, LOW}; //holds the previous button state
unsigned long lastDebounceTime[4] = {0, 0, 0, 0};
unsigned long debounceDelay = 50;

unsigned long watchdogTime = 61200; // one minute and 2 seconds
unsigned long watchdogTimer = 0;    // used to keep track of the millis() since the last rtc alarm

//*************************************************************************************************************************
//************************** Setup ****************************************************************************************
//*************************************************************************************************************************

void setup()
{
  pinMode(slaveSelectPin, OUTPUT);
  SPI.beginTransaction(SPISettings(500000, LSBFIRST, SPI_MODE0));
  SPI.begin();
  Serial.begin(9600);
  directMove(); //Set the segment to open position
  Serial.println("poop");

  for (int i = 0; i < 4; i++)
  {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }

  int8_t x = 0;
  while (x <= 1) // used to test the display on startup
  {
    timeArray[0] = x;
    timeArray[1] = x;
    timeArray[2] = x;
    timeArray[3] = x;
    directMove();
    delay(100);
    x++;
  }

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  if (rtc.lostPower())
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.disable32K();                           //we don't need the 32K Pin, so disable it
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP); // Making it so, that the alarm will trigger an interrupt
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  rtc.clearAlarm(1); // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);                              // stop oscillating signals at SQW Pin
  rtc.disableAlarm(2);                                          // turn off alarm 2 (in case it isn't off already)
  rtc.setAlarm1(DateTime(0, 0, 0, 0, 0, 00), DS3231_A1_Second); // schedule an alarm at the next minute seconds in the future
}

//*************************************************************************************************************************
//************************** Loop *****************************************************************************************
//*************************************************************************************************************************

void loop()
{

  if (rtc.alarmFired(1) || (millis() - watchdogTimer > watchdogTime))
  {                           // if the rtc alarm flag is high update the time second function is there to identify if the rtcalarm function failed
    watchdogTimer = millis(); // reset the watchdog timer
    DateTime now = rtc.now();
    rtc.clearAlarm(1);                                            // clear the alarm flag
    rtc.setAlarm1(DateTime(0, 0, 0, 0, 0, 00), DS3231_A1_Second); // reset the alarm to trigger the next minute

    Minutes = now.minute(); // Break up the time into its digits
    timeArray[1] = Minutes / 10;
    timeArray[0] = Minutes - timeArray[1] * 10;

    Hour = now.hour();
    timeArray[3] = Hour / 10;
    timeArray[2] = Hour - timeArray[3] * 10;

    if (Hour < 21 && Hour > 7)
    { // used to disable the clock at night
      offOn = true;
      directMove();
    }
    else if (offOn)
    {
      timeArray[0] = 10;
      timeArray[1] = 10;
      timeArray[2] = 10;
      timeArray[3] = 10;
      offOn = false;
    }
  }
  sleepPower(); // reduces the power consumption of the clock between minute changes
  buttonPolling();
}

//*************************************************************************************************************************
//************************** Button polling  ******************************************************************************
//*************************************************************************************************************************

void buttonPolling()
{

  for (int i = 0; i < 4; i++)
  {
    //rtc.adjust( rtc.now() + getdatetime.timedelta(minutes = 10)); // move minutes up by ten
    int reading = digitalRead(buttonPin[i]);
    if (reading != lastButtonState[i])
    {
      // reset the debouncing timer
      lastDebounceTime[i] = millis();
    }

    if (millis() - lastDebounceTime[i] > debounceDelay)
    {
      if (reading != ButtonState[i])
      {
        ButtonState[i] = reading;

        if (lastButtonState[i] == LOW)
        {
          DateTime now = rtc.now();
          Minutes = now.minute(); // Break up the time into its digits
          timeArray[1] = Minutes / 10;
          timeArray[0] = Minutes - timeArray[1] * 10;
          Hour = now.hour();
          timeArray[3] = Hour / 10;
          timeArray[2] = Hour - timeArray[3] * 10;
          int Seconds = now.minute();
          

          switch (i)
          {
          case 0:
            timeArray[0]++;
            if (timeArray[0] > 9)
            { // rollover
              timeArray[0] = 0;
            }
            Seconds = 01; //set the seconds to 1 to make the minute start when pressed
            break;
          case 1:
            timeArray[1]++;
            if (timeArray[1] > 5)
            { // rollover
              timeArray[1] = 0;
            }
            break;
          case 2:
            timeArray[2]++;
            if (timeArray[2] > 9)
            { // rollover
              timeArray[2] = 0;
            }
            break;
          case 3:
            timeArray[3]++;
            if (timeArray[3] > 2)
            { // rollover
              timeArray[3] = 0;
            }
            break;
          }

          Minutes = timeArray[1] * 10 + timeArray[0];
          Hour = timeArray[3] * 10 + timeArray[2];
          DateTime newTime = {now.year(), now.month(), now.day(), Hour, Minutes, Seconds};
          rtc.adjust(newTime);
          directMove();
          //Serial.println("pooop");

          /*char date[10] = "hh:mm:ss";
          newTime.toString(date);
          Serial.println(date);
        */
        }
      }
    }
    lastButtonState[i] = reading;
  }
}

//*************************************************************************************************************************
//************************** Sleep Power Function *************************************************************************
//*************************************************************************************************************************

void sleepPower()
{

  digitalWrite(slaveSelectPin, HIGH);
  int segmentPush = 0;
  sleepTimerNew = micros();
  while (segmentPush < 28)
  { // send out next step over spi
    if ((sleepTimerNew - sleepTimerOld) < 100)
    {
      byteOut = byteOut | stepOrder[currentStep[boardMap[segmentPush]]] << 4 * nibbleOutCount;
    }
    else if ((sleepTimerNew - sleepTimerOld) < 50)
    {
      byteOut = 0b00000000;
    }
    nibbleOutCount++;

    if (nibbleOutCount == 2)
    { // collate steps into SPI byte
      SPI.transfer(byteOut);
      byteOut = 0b00000000;
      nibbleOutCount = 0;
    }
    segmentPush++;
  }
  if ((sleepTimerNew - sleepTimerOld) >= 700)
  {
    sleepTimerOld = sleepTimerNew;
  }
  digitalWrite(slaveSelectPin, LOW);
}
//*************************************************************************************************************************
//************************** Direct Move Function *************************************************************************
//*************************************************************************************************************************
void directMove()
{

  int stepsToGo = stepsPerMove;

  while (stepsToGo > 0)
  {
    int digit = 0;
    while (digit < 4)
    {
      uint8_t oldSegments = segmentOrder[oldTimeArray[digit]];
      uint8_t newSegments = segmentOrder[timeArray[digit]];
      //Serial.print(digit,DEC);
      int segment = 0;
      while (segment < 7)
      {
        int adjustedSegment = segment + digit * 7;
        if (bitRead(oldSegments, segment) == 1)
        {
          currentStep[adjustedSegment] = currentStep[adjustedSegment] + direction[digit]; //increment in desired direction
          if (currentStep[adjustedSegment] == 9)
          {
            currentStep[adjustedSegment] = 1; //rollover
          }
          if (currentStep[adjustedSegment] == 0)
          {
            currentStep[adjustedSegment] = 8; //rollover
          }
        }
        if (bitRead(newSegments, segment) == 1)
        {
          currentStep[adjustedSegment] = currentStep[adjustedSegment] - direction[digit]; //increment in desired direction
          if (currentStep[adjustedSegment] == 9)
          {
            currentStep[adjustedSegment] = 1; //rollover
          }
          if (currentStep[adjustedSegment] == 0)
          {
            currentStep[adjustedSegment] = 8; //rollover
          }
        }
        segment++;
      }
      digit++;
    }

    digitalWrite(slaveSelectPin, HIGH);
    int segmentPush = 0;

    while (segmentPush < 28)
    { // send out next step over spi

      byteOut = byteOut | stepOrder[currentStep[boardMap[segmentPush]]] << 4 * nibbleOutCount;
      nibbleOutCount++;

      if (nibbleOutCount == 2)
      { // collate steps into SPI byte
        SPI.transfer(byteOut);
        byteOut = 0b00000000;
        nibbleOutCount = 0;
      }
      segmentPush++;
    }

    digitalWrite(slaveSelectPin, LOW);
    stepsToGo = stepsToGo - 1;
    while (micros() - timer < stepRate)
    {
      ;
    }
    timer = micros();
    digit++;
  }

  oldTimeArray[0] = timeArray[0];
  oldTimeArray[1] = timeArray[1];
  oldTimeArray[2] = timeArray[2];
  oldTimeArray[3] = timeArray[3];
}

//*************************************************************************************************************************
//************************** Alarm ISR ************************************************************************************
//*************************************************************************************************************************
void onAlarm()
{ //minuteAlarm
}
