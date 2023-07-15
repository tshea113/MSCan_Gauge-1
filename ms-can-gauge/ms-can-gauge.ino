#include <Metro.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include <EncoderTool.h>
#include <EEPROM.h>
#include <Bounce2.h>

#include "megasquirt-messages.h"
#include "pin-definitions.h"
#include "constants.h"
#include "gauge-data.h"

using namespace EncoderTool;

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
boolean connection_state = false;

// MS data vars
GaugeData gaugeData;

// Menu vars
MenuState menuState;
Settings gaugeSettings;

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
        EEPROM.write(kWarningEnableAddr, gaugeSettings.warnings_enable);
        break;
      case 1:
        EEPROM.write(kCoolantWarningAddr, gaugeSettings.coolant_warning_temp);
        EEPROM.write(kCoolantWarningAddr + 1, gaugeSettings.coolant_warning_temp >> 8);
        break;
      }
    }
    EEPROM.write(kEEPROMInitAddr, kEEPROMValidId);
  }

  gaugeSettings.warnings_enable = EEPROM.read(kWarningEnableAddr);
  gaugeSettings.coolant_warning_temp = EEPROM.read((kCoolantWarningAddr + 1)) << 8;
  gaugeSettings.coolant_warning_temp |= EEPROM.read(kCoolantWarningAddr);

  if (kDebugMode)
  {
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

  myEncoder.begin(ENC_PIN_1, ENC_PIN_2);
  encoderButton.attach(ENC_BUTTON, INPUT_PULLUP);
  encoderButton.interval(kButtonInterval);
  encoderButton.setPressedState(LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

// -------------------------------------------------------------
void loop(void)
{
  encoderButton.update();
  MenuCheck();

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

    commTimer.reset();
    connection_state = false;
  }

  // main display routine
  if ((connection_state || kDebugMode) && displayTimer.check())
  {
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

// Converts a value stored in the format (val)E-1 to a decimal string for display
String ToDecimal(int val)
{
  String temp = String(val);

  // Single digit values need a leading zero
  if (temp.length() == 1)
  {
    return "0." + temp.substring(0, temp.length());
  }
  else
  {
    return temp.substring(0, temp.length() - 1) + '.' + temp.charAt(temp.length() - 1);
  }
}

// Returns the number of characters in an integer
int IntegerWidth(int val, bool isDecimal)
{
  String temp = String(val);
  if (isDecimal)
  {
    // Single digit values need a leading zero
    if (temp.length() == 1)
    {
      return temp.length() + 2;
    }
    else
    {
      return temp.length() + 1;
    }
  }
  else
  {
    return temp.length();
  }
}

void ReadCanMessage()
{
  // ID's 1520+ are Megasquirt CAN broadcast frames
  switch (rxMessage.id)
  {
  case 1520: // 0
    gaugeData.rpm = (rxMessage.buf[6] << 8) | rxMessage.buf[7];
    break;
  case 1521: // 1
    gaugeData.spark_advance = (rxMessage.buf[0] << 8) | rxMessage.buf[1];
    gaugeData.engine = rxMessage.buf[3];
    gaugeData.afr_target = rxMessage.buf[4];
    break;
  case 1522: // 2
    gaugeData.barometer = (rxMessage.buf[0] << 8) | rxMessage.buf[1];
    gaugeData.map = (rxMessage.buf[2] << 8) | rxMessage.buf[3];
    gaugeData.mat = (rxMessage.buf[4] << 8) | rxMessage.buf[5];
    gaugeData.coolant_temp = (rxMessage.buf[6] << 8) | rxMessage.buf[7];
    // 6.895kpa = 1psi
    gaugeData.boost_psi = ((gaugeData.map - gaugeData.barometer) * 200) / 1379;
    break;
  case 1523: // 3
    gaugeData.tps = (rxMessage.buf[0] << 8) | rxMessage.buf[1];
    gaugeData.battery_voltage = (rxMessage.buf[2] << 8) | rxMessage.buf[3];
    break;
  case 1524: // 4
    gaugeData.knock = (rxMessage.buf[0] << 8) | rxMessage.buf[1];
    gaugeData.ego_correction = (rxMessage.buf[2] << 8) | rxMessage.buf[3];
    break;
  case 1526: // 6
    gaugeData.iac = (((rxMessage.buf[6] << 8) | rxMessage.buf[7]) * 49) / 125;
  case 1529: // 9
    gaugeData.dwell = (rxMessage.buf[4] << 8) | rxMessage.buf[5];
    break;
  case 1530: // 10
    gaugeData.status_1 = rxMessage.buf[0];
    gaugeData.status_2 = rxMessage.buf[1];
    gaugeData.status_3 = rxMessage.buf[2];
    gaugeData.status_6 = rxMessage.buf[6];
    gaugeData.status_7 = rxMessage.buf[7];
    break;
  case 1537: // 17
    gaugeData.boost_duty = (rxMessage.buf[4] << 8) | rxMessage.buf[5];
    break;
  case 1548: // 28
    gaugeData.idle_target = (rxMessage.buf[0] << 8) | rxMessage.buf[1];
    break;
  case 1551: // 31
    gaugeData.afr = rxMessage.buf[0];
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
    if (bitRead(gaugeData.status_2,6)) return true; // overboost
  } 
  return false;
}

void MenuCheck()
{
  // Pressing the button brings up the menu or selects position
  if (encoderButton.pressed())
  {
    if (kDebugMode) Serial.println("Button pressed!");
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
}

void DashboardView()
{
  display.clearDisplay();

  display.setTextColor(WHITE);

  int16_t labelXBound = 0;
  int16_t labelYBound = 0;
  uint16_t labelWidth = 0;
  uint16_t labelHeight = 0;

  // Dashboard Line 1
  // RPM
  display.setTextSize(1);
  display.getTextBounds(kDashboardItems[0], 0, 0, &labelXBound, &labelYBound, &labelWidth, &labelHeight);
  display.setCursor(0, (kDashboardLineHeight - 1) - labelHeight);
  display.print(kDashboardItems[0]);

  display.setTextSize(2);
  display.setCursor(labelWidth + 2, 1);
  display.println(gaugeData.rpm);

  // Dashboard Line 2
  // AFR
  display.setTextSize(1);
  display.getTextBounds(kDashboardItems[1], 0, 0, &labelXBound, &labelYBound, &labelWidth, &labelHeight);
  display.setCursor(0, ((2 * kDashboardLineHeight) - 1) - labelHeight);
  display.print(kDashboardItems[1]);

  display.setTextSize(2);
  display.setCursor(labelWidth + 2, kDashboardLineHeight + 1);
  display.print(ToDecimal(gaugeData.afr));

  // CLT
  display.setTextSize(1);
  display.getTextBounds(kDashboardItems[2], kDashboardColumnWidth, 0, &labelXBound, &labelYBound, &labelWidth, &labelHeight);
  display.setCursor(kDashboardColumnWidth, ((2 * kDashboardLineHeight) - 1) - labelHeight);
  display.print(kDashboardItems[2]);

  display.setTextSize(2);
  display.setCursor(kDashboardColumnWidth + (labelWidth + 2), kDashboardLineHeight + 1);
  display.print(gaugeData.coolant_temp / 10);

  // Dashboard Line 3
  // In boost, show boost psi, otherwise show MAT.
  if (gaugeData.map > gaugeData.barometer)
  {
    // PSI
    display.setTextSize(1);
    display.getTextBounds(kDashboardItems[5], 0, 0, &labelXBound, &labelYBound, &labelWidth, &labelHeight);
    display.setCursor(0, (2 * kDashboardLineHeight) + 1);
    display.print(kDashboardItems[5]);

    display.setTextSize(2);
    display.setCursor(labelWidth + 2, (2 * kDashboardLineHeight) + 1);
    // 6.895kpa = 1psi
    display.print(ToDecimal(gaugeData.boost_psi));
  }
  else
  {
    // MAT
    display.setTextSize(1);
    display.getTextBounds(kDashboardItems[4], 0, 0, &labelXBound, &labelYBound, &labelWidth, &labelHeight);
    display.setCursor(0, (2 * kDashboardLineHeight) + 1);
    display.print(kDashboardItems[4]);

    display.setTextSize(2);
    display.setCursor(labelWidth + 2, (2 * kDashboardLineHeight) + 1);
    display.print(gaugeData.mat / 10);
  }

  // MAP
  display.setTextSize(1);
  display.getTextBounds(kDashboardItems[3], kDashboardColumnWidth, 0, &labelXBound, &labelYBound, &labelWidth, &labelHeight);
  display.setCursor(kDashboardColumnWidth, (2 * kDashboardLineHeight) + 1);
  display.print(kDashboardItems[3]);

  display.setTextSize(2);
  display.setCursor(kDashboardColumnWidth + (labelWidth + 2), (2 * kDashboardLineHeight) + 1);
  display.print(gaugeData.map / 10);

  BottomView();
  display.display();
}

void BottomView()
{
  // Conditions for lighting up the menu items
  bool highlightItem[kNumBottomMenuItems];
  highlightItem[0] = gaugeData.check_engine_light != 0;
  highlightItem[1] = bitRead(gaugeData.status_6,6) == 1;
  highlightItem[2] = bitRead(gaugeData.status_2,7) == 1;
  highlightItem[3] = bitRead(gaugeData.status_7,4) == 1;
  highlightItem[4] = bitRead(gaugeData.status_2,6) == 1;
  highlightItem[5] = bitRead(gaugeData.engine,3) == 1;

  display.setTextSize(1);

  for (int i = 0; i < kNumBottomMenuItems; i++)
  {
    if (highlightItem[i])
    {
      display.setTextColor(BLACK, WHITE);
      display.fillRect(i * (kBottomMenuWidth - 1), ((kOledHeight - 1) - kBottomMenuHeight), kBottomMenuWidth, kBottomMenuHeight, WHITE);
    }
    else
    {
      display.setTextColor(WHITE);
      display.drawRect(i * (kBottomMenuWidth - 1), ((kOledHeight - 1) - kBottomMenuHeight), kBottomMenuWidth, kBottomMenuHeight, WHITE);
    }
    display.setCursor((i * (kBottomMenuWidth - 1)) + 2, (kOledHeight - kBottomMenuHeight) + 1);
    display.print(kBottomMenuItems[i]);
  }
}

void SingleView()
{
  // Check for rotations
  if (myEncoder.valueChanged())
  {
    encoder_index = myEncoder.getValue();
    menuState.gauge_single_position = encoder_index;

    if (kDebugMode)
    {
      Serial.print("Encoder: ");
      Serial.println(encoder_index);
      Serial.print("Single: ");
      Serial.println(menuState.gauge_single_position);
    }
  }

  String data;

  switch (static_cast<Gauges>(menuState.gauge_single_position))
  {
    case kRPMGauge:
      data = String(gaugeData.rpm, DEC);
      break;
    case kAFRGauge:
      data = ToDecimal(gaugeData.afr);
      break;
    case kCoolantGauge:
      data = ToDecimal(gaugeData.coolant_temp);
      break;
    case kMAPGauge:
      data = ToDecimal(gaugeData.map);
      break;
    case kMATGauge:
      data = ToDecimal(gaugeData.mat);
      break;
    case kBoostGauge:
      data = ToDecimal(gaugeData.boost_psi);
      break;
    case kVoltageGauge:
      data = ToDecimal(gaugeData.battery_voltage);
      break;
    case kTPSGauge:
      data = ToDecimal(gaugeData.tps);
      break;
    case kKnockGauge:
      data = ToDecimal(gaugeData.knock);
      break;
    case kBarometerGauge:
      data = ToDecimal(gaugeData.barometer);
      break;
    case kEGOCorrectionGauge:
      data = ToDecimal(gaugeData.ego_correction);
      break;
    case kIACGauge:
      data = String(gaugeData.iac, DEC);
      break;
    case kSparkDwellGauge:
      data = ToDecimal(gaugeData.dwell);
      break;
    case kBoostDutyGauge:
      data = String(gaugeData.boost_duty, DEC);
      break;
    case kIdleTargetGauge:
      data = String(gaugeData.idle_target, DEC);
      break;
    case kAfrTargetGauge:
      data = ToDecimal(gaugeData.afr_target);
      break;
    case kTimingGauge:
      data = ToDecimal(gaugeData.spark_advance);
      break;
  }

  display.clearDisplay();
  display.setTextColor(WHITE);

  // Data
  display.setTextSize(4);
  display.setCursor(((kOledWidth - 1) - (data.length() * kTextWidth4)) / 2, 0);
  display.print(data);

  // Label
  display.setTextSize(2);
  display.setCursor(0, ((kOledHeight - 1) - kTextHeight2));
  display.print(kGauges[menuState.gauge_single_position]);

  // Additional data for highest/lowest value
  display.setTextSize(2);
  switch(static_cast<Gauges>(menuState.gauge_single_position))
  {
  case kRPMGauge:
    if (gaugeData.rpm > gaugeData.rpm_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.rpm_highest = gaugeData.rpm;
      validity_window_highest = millis();
    }
    display.setCursor((kOledWidth - 1) - (IntegerWidth(gaugeData.rpm_highest, false) * kTextWidth2), (kOledHeight / 2) - 1);
    display.print(gaugeData.rpm_highest);
    break;
  case kAFRGauge:
    if (gaugeData.afr > gaugeData.afr_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.afr_highest = gaugeData.afr;
      validity_window_highest = millis();
    }
    if (gaugeData.afr < gaugeData.afr_lowest || millis() > (validity_window_lowest + kMinMaxGaugeInterval))
    {
      gaugeData.afr_lowest = gaugeData.afr;
      validity_window_lowest = millis();
    }
    display.setCursor(0, (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.afr_lowest));

    display.setCursor((kOledWidth - 1) - (IntegerWidth(gaugeData.afr_highest, true) * kTextWidth2), (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.afr_highest));
    break;
  case kCoolantGauge:
    if (gaugeData.coolant_temp > gaugeData.coolant_temp_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.coolant_temp_highest = gaugeData.coolant_temp;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (IntegerWidth(gaugeData.coolant_temp_highest, true) * kTextWidth2)), (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.coolant_temp_highest));
    break;
  case kMAPGauge:
    if (gaugeData.map > gaugeData.map_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.map_highest = gaugeData.map;
      validity_window_highest = millis();
    }
    display.setCursor((kOledWidth - 1) - (IntegerWidth(gaugeData.map_highest, true) * kTextWidth2), (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.map_highest));
    break;
  case kMATGauge:
    if (gaugeData.mat > gaugeData.mat_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.mat_highest = gaugeData.mat;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (IntegerWidth(gaugeData.mat_highest, true) * kTextWidth2)), (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.mat_highest));
    break;
  case kBoostGauge:
    if (gaugeData.boost_psi > gaugeData.boost_psi_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.boost_psi_highest = gaugeData.boost_psi;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (IntegerWidth(gaugeData.boost_psi_highest, true) * kTextWidth2)), (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.boost_psi_highest));
    break;
  case kKnockGauge:
    if (gaugeData.knock > gaugeData.knock_highest || millis() > (validity_window_highest + kMinMaxGaugeInterval))
    {
      gaugeData.knock_highest = gaugeData.knock;
      validity_window_highest = millis();
    }
    display.setCursor(((kOledWidth - 1) - (IntegerWidth(gaugeData.knock_highest, true) * kTextWidth2)), (kOledHeight / 2) - 1);
    display.print(ToDecimal(gaugeData.knock_highest));
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
      display.print(ToDecimal(gaugeData.afr));
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
