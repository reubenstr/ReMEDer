//
// ReMEDer
//
// A flashing alarm/indicator as a reminder to take medicine before bedtime.
//
// Reuben Strangelove
// Spring 2020
//
//

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RtcDS1307.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define PIN_BUTTON_TIME 5
#define PIN_BUTTON_ALARM 6
#define PIN_BUTTON_HOUR 3
#define PIN_BUTTON_MINUTE 4
#define PIN_BUTTON_RESET 8
#define PIN_LED_INDICATOR 9
#define PIN_LED_BUILTIN 13

#define DELAY_DEBOUNCE_MS 50
#define DELAY_INDICATOR_MS 10 // Rise/Decay time for indicator (ms)

// RTC Library: https://github.com/Makuna/Rtc
RtcDS1307<TwoWire> Rtc(Wire);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET 4     // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// https://github.com/JChristensen/JC_Button
#include <JC_Button.h>
Button buttonReset(PIN_BUTTON_RESET);
Button buttonTime(PIN_BUTTON_TIME);
Button buttonAlarm(PIN_BUTTON_ALARM);
Button buttonHour(PIN_BUTTON_HOUR);
Button buttonMinute(PIN_BUTTON_MINUTE);

const unsigned long
    REPEAT_FIRST(500), // ms required before repeating on long press
    REPEAT_INCR(100);  // repeat interval for long press

#define countof(a) (sizeof(a) / sizeof(a[0]))

int timeHour, timeMinute, alarmHour, alarmMinute;
int oldTimeHour, oldTimeMinute, oldAlarmHour, oldAlarmMinute;
bool indicatorOn = false;

void Error(int x)
{
  // Loop forever, indicates fatal error.
  analogWrite(PIN_LED_INDICATOR, 0);
  delay(1000);
  analogWrite(PIN_LED_INDICATOR, 127);
  delay(x);
}

void ProcessIndicator(bool isOn)
{
  static unsigned int pwmValue = 0;
  static unsigned long pwmMillis;

  if (isOn == false)
  {
    pwmValue = 0;
    analogWrite(PIN_LED_INDICATOR, pwmValue);
    return;
  }

  if ((pwmMillis + DELAY_INDICATOR_MS) < millis())
  {
    pwmMillis = millis();
    pwmValue++;
    // Sin wave (produces smooth brightness cycle). Example found from: https://forum.arduino.cc/index.php?topic=625662.0)
    analogWrite(PIN_LED_INDICATOR, (255 / 2) + (255 / 2) * sin(radians(pwmValue)));
  }
}

bool ProcessResetButton()
{
  buttonReset.read();
  return buttonReset.wasPressed();
}

bool ProcessControlButtons()
{
  static unsigned long hourRepeatCounter = REPEAT_FIRST;
  static unsigned long minuteRepeatCounter = REPEAT_FIRST;
  bool hourIncrementFlag = false;
  bool minuteIncrementFlag = false;
  bool updatePerformedFlag = false;

  buttonTime.read();
  buttonAlarm.read();
  buttonHour.read();
  buttonMinute.read();

  // Hour button:
  if (buttonHour.wasPressed())
  {
    hourIncrementFlag = true;
  }

  if (buttonHour.pressedFor(hourRepeatCounter))
  {
    hourRepeatCounter += REPEAT_INCR;
    hourIncrementFlag = true;
  }

  if (buttonHour.wasReleased())
  {
    hourRepeatCounter = REPEAT_FIRST;
  }

  if (hourIncrementFlag)
  {
    updatePerformedFlag = true;

    if (buttonTime.isPressed())
    {
      timeHour++;
      if (timeHour > 23)
      {
        timeHour = 0;
      }
    }
    else if (buttonAlarm.isPressed())
    {
      alarmHour++;
      if (alarmHour > 23)
      {
        alarmHour = 0;
      }
    }
  }

  // Minute button:
  if (buttonMinute.wasPressed())
  {
    minuteIncrementFlag = true;
  }

  if (buttonMinute.pressedFor(minuteRepeatCounter))
  {
    minuteRepeatCounter += REPEAT_INCR;
    minuteIncrementFlag = true;
  }

  if (buttonMinute.wasReleased())
  {
    minuteRepeatCounter = REPEAT_FIRST;
  }

  if (minuteIncrementFlag)
  {
    updatePerformedFlag = true;

    if (buttonTime.isPressed())
    {
      timeMinute++;
      if (timeMinute > 59)
      {
        timeMinute = 0;
      }
    }
    else if (buttonAlarm.isPressed())
    {
      alarmMinute++;
      if (alarmMinute > 59)
      {
        alarmMinute = 0;
      }
    }
  }

  return updatePerformedFlag;
}

void UpdateDisplay(int timeHour, int timeMinute, int alarmHour, int alarmMinute)
{
  char buf[20];
  //char meridiem = 'a';

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  sprintf(buf, "T: %02u:%02u", timeHour, timeMinute);
  display.println(buf);

  display.setCursor(0, 18);
  sprintf(buf, "A: %02u:%02u", alarmHour, alarmMinute);
  display.println(buf);

  display.display();
}

void printDateTime(const RtcDateTime &dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial.print(datestring);
}

void SetupRTC()
{
  // Setup code provided by library example:
  // https://github.com/Makuna/Rtc/blob/master/examples/DS1307_Simple/DS1307_Simple.ino

  Serial.print("Compile time: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  Rtc.Begin();

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if (!Rtc.IsDateTimeValid())
  {
    if (Rtc.LastError() != 0)
    {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      Serial.print("RTC communications error = ");
      Serial.println(Rtc.LastError());
      Error(100);
    }
    else
    {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println("RTC lost confidence in the DateTime!");
      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue

      Rtc.SetDateTime(compiled);
    }
  }

  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled)
  {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  }
  else if (now > compiled)
  {
    Serial.println("RTC is newer than compile time. (this is expected)");
  }
  else if (now == compiled)
  {
    Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.SetSquareWavePin(DS1307SquareWaveOut_Low);

  Serial.println("RTC setup finished.");
}

void setup()
{
  Serial.begin(9600);

  Serial.println("ReMEDer starting up...");

  pinMode(PIN_LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_INDICATOR, OUTPUT);

  buttonReset.begin();
  buttonTime.begin();
  buttonAlarm.begin();
  buttonHour.begin();
  buttonMinute.begin();

  SetupRTC();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    Error(1000);
  }

  alarmHour = EEPROM.read(0);
  alarmMinute = EEPROM.read(1);
  if (alarmHour == 255)
  {
    alarmHour = 0;
  }
  if (alarmMinute == 255)
  {
    alarmMinute = 0;
  }

  //RtcDateTime dateTime = Rtc.GetDateTime();
  //timeHour = dateTime.Hour();
  //timeMinute = dateTime.Minute();
  //UpdateDisplay(timeHour, timeMinute, alarmHour, alarmMinute);
}

void loop()
{
  
  bool updateDisplayFlag = false;

  static unsigned long builtinLedMillis;
  if ((builtinLedMillis + 500) < millis())
  {
    builtinLedMillis = millis();
    digitalWrite(PIN_LED_BUILTIN, !digitalRead(PIN_LED_BUILTIN));
  }

  if (ProcessControlButtons())
  {
    updateDisplayFlag = true;

    // Check if time was updated by the user.
    if (oldTimeHour != timeHour || oldTimeMinute != timeMinute)
    {
      Rtc.SetDateTime(RtcDateTime(2020, 1, 1, timeHour, timeMinute, 0));
    }

    if (oldAlarmHour != alarmHour || oldAlarmMinute != alarmMinute)
    {
      oldAlarmHour = alarmHour;
      oldAlarmMinute = alarmMinute;
      // TODO: add mechanism to avoid writing to many cycles to EEPROM.
      EEPROM.write(0, alarmHour);
      EEPROM.write(1, alarmMinute);
    }
  }

  // Check if time has updated.
  if (Rtc.IsDateTimeValid())
  {
    RtcDateTime dateTime = Rtc.GetDateTime();
    timeHour = dateTime.Hour();
    timeMinute = dateTime.Minute();

    if (oldTimeHour != timeHour || oldTimeMinute != timeMinute)
    {
      oldTimeHour = timeHour;
      oldTimeMinute = timeMinute;
      updateDisplayFlag = true;

      // Check for alarm trigger
      if (timeHour == alarmHour && timeMinute == alarmMinute)
      {
        indicatorOn = true;
      }
    }
  }
  else
  {
    // RTC error, likely bad battery.
    Error(100);
  }

  if (updateDisplayFlag)
  {
    updateDisplayFlag = false;
    UpdateDisplay(timeHour, timeMinute, alarmHour, alarmMinute);
  }

  if (ProcessResetButton())
  {
    indicatorOn = false;
  }

  ProcessIndicator(indicatorOn);
}