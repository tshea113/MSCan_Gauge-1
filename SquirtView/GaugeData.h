// Constants
const int16_t kNumViews                     = 4;      // Number of gauge views
const int16_t kNumSettings                  = 5;      // Number of settings
const int16_t kNumGauges                    = 16;     // Number of gauges
const int16_t kNumGraphs                    = 3;      // Number of graph gauges

const String kViews[kNumViews] = {"Dashboard", "Single", "Graph", "Settings"};
const String kSettings[kNumSettings] = {"LED Ring", "Shift RPM", "Warnings", "Coolant Warn", "Exit"};
const String kGauges[kNumGauges] = {"RPM", "AFR", "Coolant", "MAP", "MAT", "Timing", "Voltage", "TPS", "Knock", "Barometer", "EGO Corr", "IAC", "Spark Dwell", "Boost Duty", "Idle Target", "AFR Target"};
const String kGraphs[kNumGraphs] = {"AFR", "MAP", "MAT"};

enum ViewsMenu : int16_t
{
  kDashboardView = 0,
  kSingleView = 1,
  kGraphView = 2,
  kSettingView = 3
};

enum SettingMenu : int16_t
{
  kLedRingEnableSetting = 0,
  kShiftRPMSetting = 1,
  kWarningsEnableSetting = 2,
  kCoolantWarningSetting = 3,
  kExitSetting = 4
};

enum Gauges : int16_t
{
  kRPMGauge = 0,
  kAFRGauge = 1,
  kCoolantGauge = 2,
  kMAPGauge = 3,
  kMATGauge = 4,
  kTimingGauge = 5,
  kVoltageGauge = 6,
  kTPSGauge = 7,
  kKnockGauge = 8,
  kBarometerGauge = 9,
  kEGOCorrectionGauge = 10,
  kIACGauge = 11,
  kSparkDwellGauge = 12,
  kBoostDutyGauge = 13,
  kIdleTargetGauge = 14,
  kAfrTargetGauge = 15
};

enum Graphs : int16_t
{
  kAFRGraph = 0,
  kMAPGraph = 1,
  kMATGraph = 2
};

// EEPROM Addresses
const uint8_t kEEPROMInitAddr     = 0;
const uint8_t kRingEnableAddr     = 1;
const uint8_t kShiftRpmAddr       = 2;
const uint8_t kWarningEnableAddr  = 4;
const uint8_t kCoolantWarningAddr = 5;

const int16_t kEEPROMValidId = 4;

// Data Structures
struct MenuState
{
  bool inMenu = false;
  bool inSettings = false;
  bool settingSelect = false;
  uint8_t menuPos = 0;
  uint8_t settingsPos = 0;
  uint8_t gaugeSinglePos = 0;
  uint8_t gaugeGraphPos = 0;
};

struct Settings
{
  bool dirty = false;
  bool LEDRingEnable = true;
  int16_t shiftRPM = 6800;
  bool warningsEnable = true;
  int16_t coolantWarning = 240;
};

struct GaugeData
{
  int16_t RPM;
  int16_t CLT;
  int16_t MAP;
  int16_t MAT;
  int16_t SPKADV;
  int16_t BATTV;
  int16_t TPS;
  int16_t Knock;
  int16_t Baro;
  int16_t EGOc;
  int16_t IAC;
  int16_t dwell;
  int16_t bstduty;
  int16_t idle_tar;
  int16_t AFR;
  int16_t AFR_tar;
  int16_t MAP_highest;
  int16_t RPM_highest;
  int16_t CLT_highest;
  int16_t MAT_highest;
  int16_t Knock_highest;
  int16_t AFR_highest;
  int16_t AFR_lowest;
  uint8_t engine;
  uint8_t CEL;
  uint8_t status1;
  uint8_t status2;
  uint8_t status3;
  uint8_t status6;
  uint8_t status7;
};