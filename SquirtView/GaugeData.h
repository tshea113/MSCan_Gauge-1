// Constants
const int NUM_VIEWS                     = 4;      // Number of gauge views
const int NUM_SETTINGS                  = 2;      // Number of settings
const int NUM_GAUGES                    = 16;     // Number of gauges
const int NUM_GRAPHS                    = 3;      // Number of graph gauges

const String VIEWS[NUM_VIEWS] = {"Dashboard", "Single", "Graph", "Settings"};
const String SETTINGS[NUM_SETTINGS] = {"LED Ring", "Exit"};
const String GAUGES[NUM_GAUGES] = {"RPM", "AFR", "Coolant", "MAP", "MAT", "Timing", "Voltage", "TPS", "Knock", "Barometer", "EGO Corr", "IAC", "Spark Dwell", "Boost Duty", "Idle Target", "AFR Target"};
const String GRAPHS[NUM_GRAPHS] = {"AFR", "MAP", "MAT"};

// EEPROM Addresses
const uint8_t NEOPIXEL_ENABLE_ADDR  = 0;

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
};

struct GaugeData
{
  unsigned int RPM;
  unsigned int CLT;
  unsigned int MAP;
  unsigned int MAT;
  unsigned int SPKADV;
  unsigned int BATTV;
  unsigned int TPS;
  unsigned int Knock;
  unsigned int Baro;
  unsigned int EGOc;
  unsigned int IAC;
  unsigned int dwell;
  unsigned int bstduty;
  unsigned int idle_tar;
  int AFR;
  int AFR_tar;
  unsigned int MAP_highest;
  unsigned int RPM_highest;
  unsigned int CLT_highest;
  unsigned int MAT_highest;
  unsigned int Knock_highest;
  int AFR_highest;
  int AFR_lowest;
  uint8_t engine;
  uint8_t CEL;
  uint8_t status1;
  uint8_t status2;
  uint8_t status3;
  uint8_t status6;
  uint8_t status7;
};