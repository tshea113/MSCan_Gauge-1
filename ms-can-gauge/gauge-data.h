// Constants
const int kNumViews                     = 4;      // Number of gauge views
const int kNumSettings                  = 3;      // Number of settings
const int kNumGauges                    = 17;     // Number of gauges
const int kNumGraphs                    = 3;      // Number of graph gauges
const int kNumBottomMenuItems           = 6;      // Number of items in the dashboard bottom menu
const int kNumDashboardItems            = 6;      // Number of items in the dashboard

const String kViews[kNumViews] = {"Dashboard", "Single", "Graph", "Settings"};
const String kSettings[kNumSettings] = {"Warnings", "Coolant Warn", "Exit"};
const String kGauges[kNumGauges] = {"RPM", "AFR", "Coolant", "MAP", "MAT", "Boost", "Voltage", "TPS", "Knock", "Barometer", "EGO Corr", "IAC", "Sprk Dwell", "Boost Duty", "Idl Target", "AFR Target", "Timing"};
const String kGraphs[kNumGraphs] = {"AFR", "MAP", "MAT"};
const String kBottomMenuItems[kNumBottomMenuItems] = {"CEL", "Fan", "Idl", "Knk", "Bst", "WUE"};
const String kDashboardItems[kNumDashboardItems] = {"RPM", "AFR", "CLT", "MAP", "MAT", "PSI"};

enum ViewsMenu : int
{
  kDashboardView = 0,
  kSingleView = 1,
  kGraphView = 2,
  kSettingView = 3
};

enum SettingMenu : int
{
  kWarningsEnableSetting = 0,
  kCoolantWarningSetting = 1,
  kExitSetting = 2
};

enum Gauges : int
{
  kRPMGauge = 0,
  kAFRGauge = 1,
  kCoolantGauge = 2,
  kMAPGauge = 3,
  kMATGauge = 4,
  kBoostGauge = 5,
  kVoltageGauge = 6,
  kTPSGauge = 7,
  kKnockGauge = 8,
  kBarometerGauge = 9,
  kEGOCorrectionGauge = 10,
  kIACGauge = 11,
  kSparkDwellGauge = 12,
  kBoostDutyGauge = 13,
  kIdleTargetGauge = 14,
  kAfrTargetGauge = 15,
  kTimingGauge = 16
};

enum Graphs : int
{
  kAFRGraph = 0,
  kMAPGraph = 1,
  kMATGraph = 2
};

// EEPROM Addresses
const uint8_t kEEPROMInitAddr     = 0;
const uint8_t kWarningEnableAddr  = 1;
const uint8_t kCoolantWarningAddr = 2;

const int kEEPROMValidId = 2;

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
  bool warnings_enable = true;
  int coolant_warning_temp = 240;
};

struct GaugeData
{
  uint16_t rpm;
  int16_t coolant_temp;
  int16_t map;
  int16_t mat;
  int16_t boost_psi;
  int16_t spark_advance;
  int16_t battery_voltage;
  int16_t tps;
  int16_t knock;
  int16_t barometer;
  int16_t ego_correction;
  int16_t iac;
  uint16_t dwell;
  uint16_t boost_duty;
  uint16_t idle_target;
  uint16_t afr;
  uint8_t afr_target;
  int16_t map_highest;
  int16_t boost_psi_highest;
  uint16_t rpm_highest;
  int16_t coolant_temp_highest;
  int16_t mat_highest;
  int16_t knock_highest;
  uint16_t afr_highest;
  uint16_t afr_lowest;
  uint8_t engine;
  uint8_t check_engine_light;
  uint8_t status_1;
  uint8_t status_2;
  uint8_t status_3;
  uint8_t status_6;
  uint8_t status_7;
};