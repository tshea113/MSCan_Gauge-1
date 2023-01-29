// Constants
const int kNumViews                     = 4;      // Number of gauge views
const int kNumSettings                  = 5;      // Number of settings
const int kNumGauges                    = 16;     // Number of gauges
const int kNumGraphs                    = 3;      // Number of graph gauges
const int kNumBottomMenuItems           = 6;      // Number of items in the dashboard bottom menu

const String kViews[kNumViews] = {"Dashboard", "Single", "Graph", "Settings"};
const String kSettings[kNumSettings] = {"LED Ring", "Shift RPM", "Warnings", "Coolant Warn", "Exit"};
const String kGauges[kNumGauges] = {"RPM", "AFR", "Coolant", "MAP", "MAT", "Timing", "Voltage", "TPS", "Knock", "Barometer", "EGO Corr", "IAC", "Sprk Dwell", "Boost Duty", "Idl Target", "AFR Target"};
const String kGraphs[kNumGraphs] = {"AFR", "MAP", "MAT"};
const String kBottomMenuItems[kNumBottomMenuItems] = {"CEL", "Fan", "Idl", "Knk", "Bst", "WUE"};

enum ViewsMenu : int
{
  kDashboardView = 0,
  kSingleView = 1,
  kGraphView = 2,
  kSettingView = 3
};

enum SettingMenu : int
{
  kLedRingEnableSetting = 0,
  kShiftRPMSetting = 1,
  kWarningsEnableSetting = 2,
  kCoolantWarningSetting = 3,
  kExitSetting = 4
};

enum Gauges : int
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

enum Graphs : int
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

const int kEEPROMValidId = 4;

// Data Structures
struct MenuState
{
  bool in_menu = false;
  bool in_settings = false;
  bool is_setting_selected = false;
  uint8_t menu_position = 0;
  uint8_t settings_position = 0;
  uint8_t gauge_single_position = 0;
  uint8_t gauge_graph_position = 0;
};

struct Settings
{
  bool dirty = false;
  bool led_ring_enable = true;
  int shift_rpm = 6800;
  bool warnings_enable = true;
  int coolant_warning_temp = 240;
};

struct GaugeData
{
  int rpm;
  int coolant_temp;
  int map;
  int mat;
  int spark_advance;
  int battery_voltage;
  int tps;
  int knock;
  int barometer;
  int ego_correction;
  int iac;
  int dwell;
  int boost_duty;
  int idle_target;
  int afr;
  int afr_target;
  int map_highest;
  int rpm_highest;
  int coolant_temp_highest;
  int mat_highest;
  int knock_highest;
  int afr_highest;
  int afr_lowest;
  uint8_t engine;
  uint8_t check_engine_light;
  uint8_t status_1;
  uint8_t status_2;
  uint8_t status_3;
  uint8_t status_6;
  uint8_t status_7;
};