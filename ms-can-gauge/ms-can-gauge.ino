#ifdef __cplusplus
extern "C" {
#endif
  void startup_early_hook() {
    WDOG_TOVALL = 2000; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
    WDOG_TOVALH = 0;
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
    WDOG_PRESC = 0; // prescaler 
  }
#ifdef __cplusplus
}
#endif

#include <Metro.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include <FastLED.h>
#include <EncoderTool.h>
#include <EEPROM.h>
#include <Bounce2.h>

#include "megasquirt-messages.h"
#include "pin-definitions.h"
#include "constants.h"
#include "gauge-data.h"

using namespace EncoderTool;

// FastLED
CRGB leds[kNumLeds];

// Encoder
Encoder myEncoder;
Bounce2::Button encoderButton;
int encoder_index;

// FlexCAN
// Teensy 3.2 only has CAN0, but Teensy 4.0 has CAN1, CAN2, and CAN3, so we must
// set these up based upon the device for which we are compiling.
#if defined(__MK20DX256__) || defined(__MK64FX512__) // Teensy 3.2/3.5
  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> myCan;
#endif
#if defined(__IMXRT1062__) // Teensy 4.0
  FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> myCan;
#endif

// OLED Display I2C
Adafruit_SSD1306 display(kOledWidth, kOledHeight, &Wire, OLED_RESET);

// Metro ticks are milliseconds
Metro commTimer = Metro(kCanTimeout);
Metro displayTimer = Metro(kDisplayRefresh);
Metro gaugeBlinkTimer = Metro(kGaugeFlashTimer);
boolean connection_state = false;
boolean gauge_blink = false;

// MS data vars
GaugeData gaugeData;

// Menu vars
MenuState menuState;
Settings gaugeSettings;

// Watchdog timer
IntervalTimer wdTimer;

int neo_brightness = 1;
char temp_chars[11];

static CAN_message_t txMessage,rxMessage;

msg_packed rxmsg_id,txmsg_id;
msg_req_data_raw msg_req_data;

unsigned long validity_window_highest; // for hi/low + histogram window update
unsigned long validity_window_lowest;

int histogram[64];
int histogram_index;

// -------------------------------------------------------------
void setup(void)
{
  // On first run of the gauge, the EEPROM will have garbage values. To fix this,
  // we check that all values are written before starting.
  if (EEPROM.read(kEEPROMInitAddr) != kEEPROMValidId)
  {
    for(int i = EEPROM.read(kEEPROMInitAddr); i <= kEEPROMValidId; i++)
    {
      switch (i)
      {
      default:
        EEPROM.write(kRingEnableAddr, gaugeSettings.led_ring_enable);
        break;
      case 1:
        EEPROM.write(kShiftRpmAddr, gaugeSettings.shift_rpm);
        EEPROM.write(kShiftRpmAddr + 1, gaugeSettings.shift_rpm >> 8);
        break;
      case 2:
        EEPROM.write(kWarningEnableAddr, gaugeSettings.warnings_enable);
        break;
      case 3:
        EEPROM.write(kCoolantWarningAddr, gaugeSettings.coolant_warning_temp);
        EEPROM.write(kCoolantWarningAddr + 1, gaugeSettings.coolant_warning_temp >> 8);
      }
    }
    EEPROM.write(kEEPROMInitAddr, kEEPROMValidId);
  }

  gaugeSettings.led_ring_enable = EEPROM.read(kRingEnableAddr);
  gaugeSettings.shift_rpm = EEPROM.read((kShiftRpmAddr + 1)) << 8;
  gaugeSettings.shift_rpm |= EEPROM.read(kShiftRpmAddr);
  gaugeSettings.warnings_enable = EEPROM.read(kWarningEnableAddr);
  gaugeSettings.coolant_warning_temp = EEPROM.read((kCoolantWarningAddr + 1)) << 8;
  gaugeSettings.coolant_warning_temp |= EEPROM.read(kCoolantWarningAddr);

  if (kDebugMode)
  {
    Serial.print("LED Ring Enable: ");
    Serial.println(gaugeSettings.led_ring_enable);
    Serial.print("Shift RPM: ");
    Serial.println(gaugeSettings.shift_rpm);
    Serial.print("Warnings Enable: ");
    Serial.println(gaugeSettings.warnings_enable);
    Serial.print("Coolant Warning: ");
    Serial.println(gaugeSettings.coolant_warning_temp);
  }

  myCan.begin();
  myCan.setBaudRate(kCanBaud);

  // By default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, kScreenI2cAddress);

  // Show splashscreen
  display.clearDisplay();
  display.drawBitmap(0,0, miata_logo, 128, 64, 1);
  display.display();

  FastLED.addLeds<NEOPIXEL, LEDPIN>(leds, kNumLeds);

  // Ring initialization animation
  if (gaugeSettings.led_ring_enable)
  {
    for(int i = 0; i < kNumLeds; i++)
    {
      leds[i].setRGB(16,16,16);
      FastLED.show();
      delay(20);
    }
    delay(200);
    // TODO: Don't like this magic number, maybe have this tied to some brightness variable?
    for (int j = 16; j > -1; j--) 
    {
      for(int i = 0; i < kNumLeds; i++)
      {
        leds[i].setRGB(j, j, j);
      }
      FastLED.show();
      delay(20);
    }
  }

  myEncoder.begin(ENC_PIN_1, ENC_PIN_2);
  encoderButton.attach(ENC_BUTTON, INPUT_PULLUP);
  encoderButton.interval(kButtonInterval);
  encoderButton.setPressedState(LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  wdTimer.begin(KickTheDog, 500000); // kick the dog every 500msec
}

// -------------------------------------------------------------
void loop(void)
{
  if (gaugeBlinkTimer.check())
  {
    gauge_blink = !gauge_blink;
    gaugeBlinkTimer.reset();
  }

  encoderButton.update();

  // See if we have gotten any CAN messages in the last second. display an error if not
  if (commTimer.check() && !kDebugMode)
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,56);
    display.println("Waiting for data...");
    display.setCursor(0,0);
    display.display();

    NoLightView();
    commTimer.reset();
    connection_state = false;
  }

  // main display routine
  if ((connection_state && displayTimer.check()) || kDebugMode)
  {
    MenuCheck();

    if (gaugeSettings.led_ring_enable)
    {
      ShiftLightView();
    }
    else
    {
      NoLightView();
    }

    if (menuState.in_menu)
    {
      MenuView();
    }
    else if (menuState.in_settings)
    {
      SettingsView();
    }
    else
    {
      if (IsOutOfBounds() && gaugeSettings.warnings_enable)
      {
        WarningView();
      }
      else
      {
        switch (static_cast<ViewsMenu>(menuState.menu_position))
        {
        case kDashboardView:
          DashboardView();
          break;
        case kSingleView:
          SingleView();
          break;
        case kGraphView:
          GraphView();
          break;
        default:
          Serial.println("Invalid view!");
        }
      } 
    }
    display.display();
    displayTimer.reset();
  }

  // handle received CAN frames
  if (myCan.read(rxMessage) && !kDebugMode)
  {
    commTimer.reset();
    connection_state = true;

    ReadCanMessage();
  }
}

void KickTheDog()
{
  Serial.println("Kicking the dog!");
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

void DivideBy10(int val)
{
  int length;

  itoa(val, temp_chars, 10);
  length = strlen(temp_chars);

  temp_chars[length + 1] = temp_chars[length]; // null shift right
  temp_chars[length] = temp_chars[length - 1];
  temp_chars[length - 1] = '.';
}

void ReadCanMessage()
{
  // ID's 1520+ are Megasquirt CAN broadcast frames
  switch (rxMessage.id)
  {
  case 1520: // 0
    gaugeData.rpm = (int)(word(rxMessage.buf[6], rxMessage.buf[7]));
    break;
  case 1521: // 1
    gaugeData.spark_advance = (int)(word(rxMessage.buf[0], rxMessage.buf[1]));
    gaugeData.engine = rxMessage.buf[3];
    gaugeData.afr_target = (int)(word(0x00, rxMessage.buf[4]));
    break;
  case 1522: // 2
    gaugeData.barometer = (int)(word(rxMessage.buf[0], rxMessage.buf[1]));
    gaugeData.map = (int)(word(rxMessage.buf[2], rxMessage.buf[3]));
    gaugeData.mat = (int)(word(rxMessage.buf[4], rxMessage.buf[5]));
    gaugeData.coolant_temp = (int)(word(rxMessage.buf[6], rxMessage.buf[7]));
    break;
  case 1523: // 3
    gaugeData.tps = (int)(word(rxMessage.buf[0], rxMessage.buf[1]));
    gaugeData.battery_voltage = (int)(word(rxMessage.buf[2], rxMessage.buf[3]));
    break;
  case 1524: // 4
    gaugeData.knock = (int)(word(rxMessage.buf[0], rxMessage.buf[1]));
    gaugeData.ego_correction = (int)(word(rxMessage.buf[2], rxMessage.buf[3]));
    break;
  case 1526: // 6
    gaugeData.iac = ((int)(word(rxMessage.buf[6], rxMessage.buf[7])) * 49) / 125;
  case 1529: // 9
    gaugeData.dwell = (int)(word(rxMessage.buf[4], rxMessage.buf[5]));
    break;
  case 1530: // 10
    gaugeData.status_1 = rxMessage.buf[0];
    gaugeData.status_2 = rxMessage.buf[1];
    gaugeData.status_3 = rxMessage.buf[2];
    gaugeData.status_6 = rxMessage.buf[6];
    gaugeData.status_7 = rxMessage.buf[7];
    break;
  case 1537: // 17
    gaugeData.boost_duty = (int)(word(rxMessage.buf[4], rxMessage.buf[5]));
    break;
  case 1548: // 28
    gaugeData.idle_target = (int)(word(rxMessage.buf[0], rxMessage.buf[1]));
    break;
  case 1551: // 31
    gaugeData.afr = (int)(word(0x00, rxMessage.buf[0]));
    break;
  case 1574: // 54
    gaugeData.check_engine_light = rxMessage.buf[2];
    break;
  default: 
    // not a broadcast packet
    // assume this is a normal Megasquirt CAN protocol packet and decode the header
    if (rxMessage.flags.extended)
    {
      rxmsg_id.i = rxMessage.id;
      // is this being sent to us?
      if (rxmsg_id.values.to_id == kMyCanId)
      {
        switch (rxmsg_id.values.msg_type)
        {
        case 1: // MSG_REQ - request data
          // the data required for the MSG_RSP header is packed into the first 3 data bytes
          msg_req_data.bytes.b0 = rxMessage.buf[0];
          msg_req_data.bytes.b1 = rxMessage.buf[1];
          msg_req_data.bytes.b2 = rxMessage.buf[2];
          // Create the tx packet header
          txmsg_id.values.msg_type = 2; // MSG_RSP
          txmsg_id.values.to_id = kMsCanId; // Megasquirt CAN ID should normally be 0
          txmsg_id.values.from_id = kMyCanId;
          txmsg_id.values.block = msg_req_data.values.varblk;
          txmsg_id.values.offset = msg_req_data.values.varoffset;
          txMessage.flags.extended = 1;
          txMessage.id = txmsg_id.i;
          txMessage.len = 8;
          // Use the same block and offset as JBPerf IO expander board for compatibility reasons
          // Docs at http://www.jbperf.com/io_extender/firmware/0_1_2/io_extender.ini (or latest version)

          // realtime clock
          if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 110)
          {
            /*
              rtc_sec          = scalar, U08,  110, "", 1,0
              rtc_min          = scalar, U08,  111, "", 1,0
              rtc_hour         = scalar, U08,  112, "", 1,0
              rtc_day          = scalar, U08,  113, "", 1,0 // not sure what "day" means. seems to be ignored...
              rtc_date         = scalar, U08,  114, "", 1,0
              rtc_month        = scalar, U08,  115, "", 1,0
              rtc_year         = scalar, U16,  116, "", 1,0
            */

            // only return clock info if the local clock has actually been set (via GPS or RTC)
            if (timeStatus() == timeSet)
            {
              txMessage.buf[0] = second();
              txMessage.buf[1] = minute();
              txMessage.buf[2] = hour();
              txMessage.buf[3] = 0;
              txMessage.buf[4] = day();
              txMessage.buf[5] = month();
              txMessage.buf[6] = year() / 256;
              txMessage.buf[7] = year() % 256;
              // send the message!
              myCan.write(txMessage);
            }
          } 
        }
      }
    }
    else
    {
      Serial.write("ID: ");
      Serial.print(rxMessage.id);
    }
  }
}

boolean IsOutOfBounds()
{
  if (gaugeData.rpm > 100)
  {
    if ((gaugeData.coolant_temp/10) > gaugeSettings.coolant_warning_temp) return true;
    // if (gaugeData.check_engine_light != 0) return true;
    if (bitRead(gaugeData.status_2,6)) return true; // overboost
  } 
  return false;
}

void MenuCheck()
{
  // Pressing the button brings up the menu or selects position
  if (encoderButton.pressed())
  {
    // Settings Menu
    if (menuState.in_settings)
    {
      if (menuState.is_setting_selected)
      {
        menuState.is_setting_selected = false;
        myEncoder.setLimits(0, kNumSettings - 1, true);
        myEncoder.setValue(menuState.settings_position);
      }
      else
      {
        menuState.is_setting_selected = true;
        switch (static_cast<SettingMenu>(menuState.settings_position))
        {
        case kLedRingEnableSetting:
          myEncoder.setLimits(0, 1, true);
          myEncoder.setValue(gaugeSettings.led_ring_enable);
          break;
        case kShiftRPMSetting:
          myEncoder.setLimits(kMinRpm/kRpmInterval, kMaxRpm/kRpmInterval, true);
          myEncoder.setValue(gaugeSettings.shift_rpm/kRpmInterval);
          break;
        case kWarningsEnableSetting:
          myEncoder.setLimits(0, 1, true);
          myEncoder.setValue(gaugeSettings.warnings_enable);
          break;
        case kCoolantWarningSetting:
          myEncoder.setLimits(0, kMaxCoolantTemp, true);
          myEncoder.setValue(gaugeSettings.coolant_warning_temp);
          break;
        case kExitSetting:
          menuState.settings_position = 0;
          menuState.in_settings = false;
          menuState.is_setting_selected = false;
          menuState.in_menu = true;
          myEncoder.setLimits(0, kNumViews - 1, true);
          myEncoder.setValue(menuState.menu_position);
          WriteSettingsToEEPROM();
          break;
        }
      }
    }
    // Views Menu
    else
    {
      if (menuState.in_menu)
      {
        menuState.in_menu = false;
        switch (static_cast<ViewsMenu>(menuState.menu_position))
        {
        case kDashboardView:
          break;
        case kSingleView:
          myEncoder.setLimits(0, kNumGauges - 1, true);
          myEncoder.setValue(menuState.gauge_single_position);
          break;
        case kGraphView:
          myEncoder.setLimits(0, kNumGraphs - 1, true);
          myEncoder.setValue(menuState.gauge_graph_position);
          break;
        case kSettingView:
          menuState.in_settings = true;
          myEncoder.setLimits(0, kNumSettings - 1, true);
          myEncoder.setValue(0);
          break;
        }
      }
      else
      {
        menuState.in_menu = true;
        myEncoder.setLimits(0, kNumViews - 1, true);
        myEncoder.setValue(menuState.menu_position);
      }
    }
  }
}

void WriteSettingsToEEPROM()
{
  if (gaugeSettings.dirty)
  {
    if (kDebugMode)
    {
      Serial.println("Writing to EEPROM!");
    }

    EEPROM.write(kRingEnableAddr, gaugeSettings.led_ring_enable);
    EEPROM.write(kShiftRpmAddr, gaugeSettings.shift_rpm);
    EEPROM.write(kShiftRpmAddr + 1, gaugeSettings.shift_rpm >> 8);
    EEPROM.write(kWarningEnableAddr, gaugeSettings.warnings_enable);
    EEPROM.write(kCoolantWarningAddr, gaugeSettings.coolant_warning_temp);
    EEPROM.write(kCoolantWarningAddr + 1, gaugeSettings.coolant_warning_temp >> 8);
    gaugeSettings.dirty = false;

    if (kDebugMode)
    {
      Serial.println("Done writing to EEPROM!");
    }
  }
}

void MenuView()
{
  // Check for rotations
  if (myEncoder.valueChanged())
  {
    encoder_index = myEncoder.getValue();
    menuState.menu_position = encoder_index;

    if (kDebugMode)
    {
      Serial.print("Encoder: ");
      Serial.println(encoder_index);
      Serial.print("Menu: ");
      Serial.println(menuState.menu_position);
    }
  }

  display.clearDisplay();
  PrintMenu(kViews, kNumViews, menuState.menu_position);
  display.display();
}

void SettingsView()
{
  // Check for rotations
  if (myEncoder.valueChanged())
  {
    encoder_index=myEncoder.getValue();
    if (menuState.is_setting_selected)
    {
      switch (static_cast<SettingMenu>(menuState.settings_position))
      {
      case kLedRingEnableSetting:
        gaugeSettings.led_ring_enable = encoder_index;
        break;
      case kShiftRPMSetting:
        gaugeSettings.shift_rpm = encoder_index * kRpmInterval;
        break;
      case kWarningsEnableSetting:
        gaugeSettings.warnings_enable = encoder_index;
        break;
      case kCoolantWarningSetting:
        gaugeSettings.coolant_warning_temp = encoder_index;
        break;
      case kExitSetting:
        break;
      }
      gaugeSettings.dirty = true;      
    }
    else
    {
      menuState.settings_position = encoder_index;
    }

    if (kDebugMode)
    {
      Serial.print("Encoder: ");
      Serial.println(encoder_index);
      Serial.print("Settings: ");
      Serial.println(menuState.settings_position);
    }
  }

  display.clearDisplay();

  display.setTextSize(1);
  for (int i = 0; i < kNumSettings; i++)
  {
    if (menuState.settings_position == i && !menuState.is_setting_selected)
    {
      display.setTextColor(BLACK, WHITE);
    }
    else
    {
      display.setTextColor(WHITE);
    }

    display.setCursor(2, (10 * i) + 2);
    display.print(kSettings[i]);

    if (menuState.settings_position == i && menuState.is_setting_selected)
    {
      display.setTextColor(BLACK, WHITE);   
    }
    else
    {
      display.setTextColor(WHITE);       
    }

    display.setCursor(100, (10 * i) + 2);
    switch (static_cast<SettingMenu>(i))
    {
    case kLedRingEnableSetting:
      display.print(gaugeSettings.led_ring_enable ? "On" : "Off");
      break;
    case kShiftRPMSetting:
      display.print(gaugeSettings.shift_rpm);
      break;
    case kWarningsEnableSetting:
      display.print(gaugeSettings.warnings_enable ? "On" : "Off"); 
      break;
    case kCoolantWarningSetting:
      display.print(gaugeSettings.coolant_warning_temp);
      break;
    case kExitSetting:
      break;
    }
  }

  display.display();
}

void WarningView()
{
  int dlength;
  int midpos;

  display.clearDisplay();

  if ((gaugeData.coolant_temp/10) > gaugeSettings.coolant_warning_temp)
  {
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("WARNING");

    dlength = 3;
    midpos = (63 - ((dlength * 23) / 2));
    display.setCursor(midpos, 16);
    display.setTextSize(4);
    display.print(gaugeData.coolant_temp / 10);

    display.setTextSize(2);
    display.setCursor(0, 48);
    display.print("CLT");
  }

  display.display();

  // if (bitRead(gaugeData.check_engine_light, 0))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("MAP");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");
  // }

  // if (bitRead(gaugeData.check_engine_light, 1))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("MAT");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");

  // }

  // if (bitRead(gaugeData.check_engine_light, 2))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("CLT");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");
  // }

  // if (bitRead(gaugeData.check_engine_light, 3))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("TPS");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");

  // }

  // if (bitRead(gaugeData.check_engine_light, 4))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("BATT");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");
  // }

  // if (bitRead(gaugeData.check_engine_light, 5))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("AFR");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");
  // }

  // if (bitRead(gaugeData.check_engine_light, 6))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("Sync");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");
  // }

  // if (bitRead(gaugeData.check_engine_light, 7))
  // {
  //   display.setTextColor(WHITE);
  //   dlength=3;
  //   midpos=(63 - ((dlength * 23) / 2));
  //   display.setCursor(29, 0);
  //   display.setTextSize(4);
  //   display.print("EGT");
  //   display.setTextSize(2);
  //   display.setCursor(8, 48);
  //   display.print("Error");
  // }

  // if ( bitRead(gaugeData.status_2,6) == 1)
  // {
  //   DangerView();
  // }
}

void DashboardView()
{
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  display.setCursor(41, 0); // 4 char normally - 4 * 10 = 40, - 128 = 88, /2 = 44
  display.println(gaugeData.rpm);
  display.setTextSize(1);
  display.setCursor(21, 7);
  display.print("RPM");

  //line2
  display.setCursor(0, 26);
  display.setTextSize(1);
  display.print("AFR");
  display.setCursor(20, 19);
  display.setTextSize(2);
  DivideBy10(gaugeData.afr);
  display.print(temp_chars);

  display.setCursor(72, 25);
  display.setTextSize(1);
  display.print("CLT");
  display.setCursor(92, 18);
  display.setTextSize(2);
  display.print(gaugeData.coolant_temp / 10);

  //line3
  display.setCursor(0, 40);
  display.setTextSize(1);
  display.print("MAP");
  display.setCursor(20, 40);
  display.setTextSize(2);
  display.print(gaugeData.map / 10);

  // contextual gauge - if idle on, show IAC%
  if ( bitRead(gaugeData.status_2, 7) == 1)
  {
    display.setCursor(72, 40);
    display.setTextSize(1);
    display.print("IAC");
    display.setCursor(92, 40);
    display.setTextSize(2);
    display.print(gaugeData.iac);
  }
  else if (gaugeData.map > gaugeData.barometer)
  {
    int psi;
    display.setCursor(72, 38);
    display.setTextSize(1);
    display.print("PSI");
    display.setCursor(92, 38);
    display.setTextSize(1);
    // 6.895kpa = 1psi
    psi = gaugeData.map - gaugeData.barometer;
    psi=(psi * 200) / 1379;
    DivideBy10(psi);
    display.print(temp_chars);

    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("MAT");
    display.setCursor(92, 47);
    display.print(gaugeData.mat / 10);

  }
  else
  {
    display.setCursor(72, 40);
    display.setTextSize(1);
    display.print("MAT");
    display.setCursor(92, 40);
    display.setTextSize(2);
    display.print(gaugeData.mat / 10);
  }

  BottomView();
}

void BottomView()
{
  display.setTextSize(1);
  display.drawFastHLine(1, (63 - 7), 126, WHITE);
  display.setCursor(0, 57);
  display.setTextColor(BLACK, WHITE);

  //CEL
  if ( gaugeData.check_engine_light != 0 )
  {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(2, 57, 8, WHITE);
  }
  else
  {
    display.setTextColor(WHITE);
  }
  display.setCursor(3, 57);
  display.print("CEL");
  display.drawFastVLine(1, 57, 8, WHITE);

  //Fan
  if ( bitRead(gaugeData.status_6,6) == 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(23, 57, 8, WHITE);
    display.drawFastVLine(22, 57, 8, WHITE);
    display.drawFastVLine(42, 57, 8, WHITE);
  }
  else
  {
    display.setTextColor(WHITE);
  }
  display.setCursor(24, 57);
  display.print("Fan");
  display.drawFastVLine(21, 57, 8, WHITE);

  //Idle
  if ( bitRead(gaugeData.status_2,7) == 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(44, 57, 8, WHITE);
  }
  else
  {
    display.setTextColor(WHITE);
  }
  display.setCursor(45, 57);
  display.print("Idl");
  display.drawFastVLine(43, 57, 8, WHITE);

  //Knock
  if ( bitRead(gaugeData.status_7,4) == 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(65, 57, 8, WHITE);
    display.drawFastVLine(64, 57, 8, WHITE);
    display.drawFastVLine(84, 57, 8, WHITE);
  }
  else
  {
    display.setTextColor(WHITE);
  }
  display.setCursor(66, 57);
  display.print("Knk");
  display.drawFastVLine(63, 57, 8, WHITE);

  //Overboost
  if ( bitRead(gaugeData.status_2,6) == 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(87, 57, 8, WHITE);
    display.drawFastVLine(86, 57, 8, WHITE);
  }
  else
  {
    display.setTextColor(WHITE);
  }
  display.setCursor(88, 57);
  display.print("Bst");
  display.drawFastVLine(85, 57, 8, WHITE);

  //WUE
  if ( bitRead(gaugeData.engine,3) == 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(107, 57, 8, WHITE);
  }
  else
  {
    display.setTextColor(WHITE);
  }
  display.setCursor(108, 57);
  display.print("WUE");
  display.drawFastVLine(106, 57, 8, WHITE);
  display.drawFastVLine(126, 57, 8, WHITE);

  // FAN, WUE, ASE, CEL, Idl, Knk, over boost
  // CEL - Idl - FAN - KnK - BST - AFR
  display.display();
}

void SingleView()
{
  char data[10];
  String label;
  display.clearDisplay();

  // Check for rotations
  if (myEncoder.valueChanged())
  {
    encoder_index=myEncoder.getValue();
    menuState.gauge_single_position = encoder_index;

    if (kDebugMode)
    {
      Serial.print("Encoder: ");
      Serial.println(encoder_index);
      Serial.print("Single: ");
      Serial.println(menuState.gauge_single_position);
    }
  }

  label=kGauges[menuState.gauge_single_position];
  switch (static_cast<Gauges>(menuState.gauge_single_position))
  {
    case kRPMGauge:
      itoa(gaugeData.rpm, data, 10);
      break;
    case kAFRGauge:
      DivideBy10(gaugeData.afr);
      strcpy(data, temp_chars);
      break;
    case kCoolantGauge:
      DivideBy10(gaugeData.coolant_temp);
      strcpy(data, temp_chars);
      break;
    case kMAPGauge:
      DivideBy10(gaugeData.map);
      strcpy(data, temp_chars);
      break;
    case kMATGauge:
      DivideBy10(gaugeData.mat);
      strcpy(data, temp_chars);
      break;
    case kTimingGauge:
      DivideBy10(gaugeData.spark_advance);
      strcpy(data, temp_chars);
      break;
    case kVoltageGauge:
      DivideBy10(gaugeData.battery_voltage);
      strcpy(data, temp_chars);
      break;
    case kTPSGauge:
      DivideBy10(gaugeData.tps);
      strcpy(data, temp_chars);
      break;
    case kKnockGauge:
      DivideBy10(gaugeData.knock);
      strcpy(data, temp_chars);
      break;
    case kBarometerGauge:
      DivideBy10(gaugeData.barometer);
      strcpy(data, temp_chars);
      break;
    case kEGOCorrectionGauge:
      DivideBy10(gaugeData.ego_correction);
      strcpy(data, temp_chars);
      break;
    case kIACGauge:
      itoa(gaugeData.iac, data, 10);
      break;
    case kSparkDwellGauge:
      DivideBy10(gaugeData.dwell);
      strcpy(data, temp_chars);
      break;
    case kBoostDutyGauge:
      itoa(gaugeData.boost_duty, data, 10);
      break;
    case kIdleTargetGauge:
      itoa(gaugeData.idle_target, data, 10);
      break;
    case kAfrTargetGauge:
      DivideBy10(gaugeData.afr_target);
      strcpy(data, temp_chars);
      break;
  }
  // TODO: I'm not sure if this is even needed. Need to figure out if this data is useful. If so the data should loop properly. Maybe update the UI for this a bit?
  // else
  // {
  //   temp_index = encoder_index - 15;
  //   char temporary[15];
  //   byte sbyte, bitp, dbit;
  //   strcpy_P(temporary, MSDataBin[temp_index].name);
  //   label=temporary;

  //   sbyte=pgm_read_byte(&MSDataBin[temp_index].sbyte);
  //   bitp=pgm_read_byte(&MSDataBin[temp_index].bitp);
  //   dbit=bitRead(indicator[sbyte], bitp);
  //   if ( dbit == 1 )
  //   {
  //     data[0]='O';
  //     data[1]='n';
  //     data[2]='\0';
  //   }
  //   else
  //   {
  //     data[0]='O';
  //     data[1]='f';
  //     data[2]='f';
  //     data[3]='\0';
  //   }
  // }

  int dlength = strlen(data);
  int midpos = ((kOledHeight - 1) - ((dlength * kTextWidth4) / 2));

  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(midpos, 0);
  display.print(data);

  display.setTextSize(2);
  display.setCursor(0, ((kOledHeight - 1) - kTextHeight2));
  display.print(label);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(menuState.gauge_single_position + 1);

  display.setTextSize(2);

  //Additional data for highest/lowest value
  switch(static_cast<Gauges>(menuState.gauge_single_position))
  {
  case kRPMGauge:
    if (gaugeData.rpm > gaugeData.rpm_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.rpm_highest = gaugeData.rpm;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (dlength * kTextWidth2)), (kOledHeight / 2) - 1);
    display.print(gaugeData.rpm_highest);
    break;
  case kAFRGauge:
    if (gaugeData.afr > gaugeData.afr_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.afr_highest = gaugeData.afr;
      validity_window_highest=millis();
    }
    if (gaugeData.afr < gaugeData.afr_lowest || millis() > (validity_window_lowest + kMinMaxGaugeInterval))
    {
      gaugeData.afr_lowest = gaugeData.afr;
      validity_window_lowest=millis();
    }
    display.setCursor(0, (kOledHeight / 2) - 1);
    DivideBy10(gaugeData.afr_lowest);
    display.print(temp_chars);
    display.setCursor(((kOledWidth - 1) - (dlength * kTextWidth2)), (kOledHeight / 2) - 1);
    DivideBy10(gaugeData.afr_highest);
    display.print(temp_chars);
    break;
  case kCoolantGauge:
    if (gaugeData.coolant_temp > gaugeData.coolant_temp_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.coolant_temp_highest = gaugeData.coolant_temp;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (dlength * kTextWidth2)), (kOledHeight / 2) - 1);
    DivideBy10(gaugeData.coolant_temp_highest);
    display.print(temp_chars);
    break;
  case kMAPGauge:
    if (gaugeData.map > gaugeData.map_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.map_highest = gaugeData.map;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (dlength * kTextWidth2)), (kOledHeight / 2) - 1);
    DivideBy10(gaugeData.map_highest);
    display.print(temp_chars);
    break;
  case kMATGauge:
    if (gaugeData.mat > gaugeData.mat_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.mat_highest = gaugeData.mat;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (dlength * kTextWidth2)), (kOledHeight / 2) - 1);
    DivideBy10(gaugeData.mat_highest);
    display.print(temp_chars);
    break;
  case kKnockGauge:
    if (gaugeData.knock > gaugeData.knock_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.knock_highest = gaugeData.knock;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (dlength * kTextWidth2)), (kOledHeight / 2) - 1);
    DivideBy10(gaugeData.knock_highest);
    display.print(temp_chars);
    break;
  default:
    break;
  }

  display.display();
}

void GraphView()
{
  int val;
  
  // Check for rotations
  if (myEncoder.valueChanged())
  {
    encoder_index = myEncoder.getValue();
    menuState.gauge_graph_position = encoder_index;

    if (kDebugMode)
    {
      Serial.print("Encoder: ");
      Serial.println(encoder_index);
      Serial.print("Graph: ");
      Serial.println(menuState.gauge_graph_position);
    }
  }

  // 10hz update time
  if (millis() > (validity_window_highest + 80))
  {
    display.clearDisplay();

    // TODO: The values here are going to be within some expected range. Probably could
    // improve the normalization algorithm here. As of now I don't really care about this
    // view enough to do anything about it.
    switch (static_cast<Graphs>(menuState.gauge_graph_position))
    {
    // 0-50 value normalization
    case kAFRGraph:
      val = (gaugeData.afr - 100) / 2;  // real rough estimation here here of afr on a 0-50 scale
      break;
    case kMAPGraph:
      val = ((gaugeData.map / 10) - 30) / 4;
      break;
    case kMATGraph:
      val = (gaugeData.mat / 10) / 4;
      break;
    default:
      val = 0;
    }

    if (val > 50)
    {
      val = 50;
    }

    histogram_index++;
    if (histogram_index >= 64)
    {
      histogram_index = 0;
    }
    histogram[histogram_index] = val;

    for (byte i = 0; i < 64; i++)
    {
      int x = histogram_index - i;
      if ( x < 0)
      {
        x = 64 + histogram_index - i;
      }

      // Draw a line two pixels wide for each data point
      display.drawFastVLine((128 - (i * 2)), (64 - histogram[x]), 64, WHITE);
      display.drawFastVLine((127 - (i * 2)), (64 - histogram[x]), 64, WHITE);
    }

    display.setCursor(20, 0);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print(kGraphs[menuState.gauge_graph_position]);
    display.print(" ");

    switch (static_cast<Graphs>(menuState.gauge_graph_position))
    {
    case kAFRGraph:
      DivideBy10(gaugeData.afr);
      display.print(temp_chars);
      display.drawFastHLine(0, 40, 128, WHITE); // stoich 14.7 line
      for (int x = 1; x < 128; x += 2)
      {
        display.drawPixel(x, 40, BLACK);
      }
      break;
    case kMAPGraph:
      display.print(gaugeData.map/10);
      display.drawFastHLine(0, 47, 128, WHITE); // Baro line.. roughly 98kpa
      for (int x = 1; x < 128; x += 2)
      {
        display.drawPixel(x, 47, BLACK);
      }
      break;
    case kMATGraph:
      display.print(gaugeData.mat / 10);
      break;
    }

    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print(menuState.gauge_graph_position + 1);

    validity_window_highest = millis();
  }
}

void DangerView()
{
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0,0);

  display.setCursor(4,0);
  display.setTextSize(2);
  display.print("Warning");
  display.print("!");
  display.print("!");
  display.print("!");

  display.setCursor(10,28);
  display.print("Danger to");
  display.setCursor(12,45);
  display.println("Manifold");
  display.display();
}

void ShiftLightView()
{
  if (gaugeData.rpm > 4000)
  {
    if (gaugeData.rpm < gaugeSettings.shift_rpm)
    {
      uint8_t currLights = (gaugeData.rpm - 4000) / ((gaugeSettings.shift_rpm - 4000) / kNumLeds);

      for (int i = 0; i < currLights; i++)
      {
        uint8_t red = (255 * neo_brightness) / 16;
        leds[i].setRGB(red, 0, 0);
      }
    }
    else
    {
      for (int i = 0; i < kNumLeds; i++)
      {
        if (gauge_blink)
        {
          uint8_t red = (255 * neo_brightness) / 16;
          leds[i].setRGB(red, 0, 0);
        }
        else
        {
          leds[i].setRGB(0, 0, 0);
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < kNumLeds; i++)
    {
      leds[i].setRGB(0, 0, 0);
    }
  }
  FastLED.show();
}

void WarningLightView()
{
  for (int i = 0; i < kNumLeds; i++)
  {
    if (gauge_blink)
    {
      uint8_t red = (255 * neo_brightness) / 16;
      leds[i].setRGB(red, 0, 0);
    }
    else
    {
      leds[i].setRGB(0, 0, 0);
    }
  }
  FastLED.show();
}

void NoLightView()
{
  for(int i = 0; i < kNumLeds; i++)
  {
    leds[i].setRGB(0, 0, 0);
  }
  FastLED.show();
}

void PrintMenu(const String items[], int numItems, int currPos)
{
  display.setTextSize(1);
  for (int i = 0; i < numItems; i++)
  {
    if (currPos == i)
    {
      display.setTextColor(BLACK, WHITE);
      display.setCursor(2, (10 * i) + 2);
      display.print(items[i]);
    }
    else
    {
      display.setTextColor(WHITE);
      display.setCursor(2, (10 * i) + 2);
      display.print(items[i]);
    }
  }
}
