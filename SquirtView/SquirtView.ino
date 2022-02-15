#include <Metro.h>
#include <FlexCAN.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include <FastLED.h>
#include <Encoder.h>

#include "RotKnob.h"
#include "MegasquirtMessages.h"
#include "definitions.h"
#include "constants.h"

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

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

// FastLED
CRGB leds[NUM_LEDS];

// Encoder
rotKnob<ENC_PIN_1, ENC_PIN_2> myEnc;
volatile unsigned long last_millis;   //switch debouncing

// OLED Display Hardware SPI
// TODO: New screen uses SPI fix this
// Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Metro ticks are milliseconds
Metro commTimer = Metro(CAN_TIMEOUT);
Metro displayTimer = Metro(DISPLAY_REFRESH);
Metro ledTimer = Metro(LED_FLASH_TIMER);
Metro gaugeBlinkTimer = Metro(GAUGE_FLASH_TIMER);
boolean connectionState = false;
boolean gaugeBlink = false;

//MS data vars
GaugeData gaugeData;

uint8_t R_index = 0;  // for rotary encoder
byte B_index = 0;     // Button increment
byte M_index = 0;     // Menu increment
byte S_index = 0;     // Select increment

byte neo_brightness = 4;
byte g_textsize = 1;
char tempchars[11];

// -------------------------------------------------------------
static void ledBlink()
{
  ledTimer.reset();
  digitalWrite(TEENSY_LED, 1);
}

static CAN_message_t txmsg,rxmsg;

msg_packed rxmsg_id,txmsg_id;
msg_req_data_raw msg_req_data;

unsigned long validity_window; // for hi/low + histogram window update
unsigned long validity_window2;

byte histogram[64]; // 512 memory usage
byte histogram_index;

// -------------------------------------------------------------
void setup(void)
{
  SPI.setSCK(SPI_SCK); // alternate clock pin so we can still use the LED
  pinMode(TEENSY_LED, OUTPUT);
  digitalWrite(TEENSY_LED, 1);

  Can0.begin(CAN_BAUD);

  // Set encoder pins as input with internal pull-up resistors enabled
  pinMode(RBUTTON_INT, INPUT);
  digitalWrite(RBUTTON_INT, HIGH);
  attachInterrupt(RBUTTON_INT, ISR_debounce, FALLING);

  FastLED.show();

  // By default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  // TODO: Revert when SPI screen is used.
  // display.begin(SSD1306_SWITCHCAPVCC);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Show splashscreen
  display.clearDisplay();
  display.drawBitmap(0,0, miata_logo, 128, 64, 1);
  display.display();

  FastLED.addLeds<NEOPIXEL, LEDPIN>(leds, NUM_LEDS);
  
  // Ring initialization animation
  for(int i = 0; i < NUM_LEDS; i++)
  {
    leds[i].setRGB(16,16,16);
    FastLED.show();
    delay(20);
  }
  delay(200);
  for (int j = NUM_LEDS; j > -1; j--) 
  {
    for(int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].setRGB(j, j, j);
    }
    FastLED.show();
    delay(20);
  }

  // TODO: This value is chosen because of the number of single gauges. Probably a better way to do this.
  // Encoder has values from 0 to 15
  myEnc.begin(0, 0, 15);

  delay(1000);
  digitalWrite(TEENSY_LED, 0);
}

// -------------------------------------------------------------
void loop(void)
{
  if (ledTimer.check() && digitalRead(TEENSY_LED))
  {
    digitalWrite(TEENSY_LED, 0);
    ledTimer.reset();
  }
  if (gaugeBlinkTimer.check())
  {
    gaugeBlink = gaugeBlink ^ 1;
    gaugeBlinkTimer.reset();
  }

  // See if we have gotten any CAN messages in the last second. display an error if not
  if (commTimer.check())
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,56);
    display.println("Waiting for data...");
    display.setCursor(0,0);
    display.display();

    for(int i = 0; i < 16; i++)
    {
      leds[i].setRGB(0, 0, 0); // initialize led ring
    }
    FastLED.show();
    commTimer.reset();
    connectionState = false;
  }

  // main display routine
  if (connectionState && displayTimer.check())
  {
    if (myEnc.available())
    {
      R_index=myEnc.read();
    }
    if (! value_oob() )
    {
      switch (B_index)
      {
      case 0:
        gauge_vitals();
        break;
      case 1:
        gauge_single();
        break;
      case 2:
        gauge_histogram();
        break;
      case 3:
        gauge_debug();
        break;
      case 4:
        gauge_menu();
        break;
      }
      write_neopixel();

    } 
    else
    {
      gauge_warning();
      FastLED.show();
    }
    display.display();
    displayTimer.reset();
  }

  // handle received CAN frames
  if ( Can0.read(rxmsg) )
  {
    commTimer.reset();
    connectionState = true;
    ledBlink();

    // ID's 1520+ are Megasquirt CAN broadcast frames
    switch (rxmsg.id)
    {
      case 1520: // 0
        gaugeData.RPM = (int)(word(rxmsg.buf[6], rxmsg.buf[7]));
        break;
      case 1521: // 1
        gaugeData.SPKADV = (int)(word(rxmsg.buf[0], rxmsg.buf[1]));
        gaugeData.engine = rxmsg.buf[3];
        gaugeData.AFR_tar = (int)(word(0x00, rxmsg.buf[4]));
        break;
      case 1522: // 2
        gaugeData.Baro = (int)(word(rxmsg.buf[0], rxmsg.buf[1]));
        gaugeData.MAP = (int)(word(rxmsg.buf[2], rxmsg.buf[3]));
        gaugeData.MAT = (int)(word(rxmsg.buf[4], rxmsg.buf[5]));
        gaugeData.CLT = (int)(word(rxmsg.buf[6], rxmsg.buf[7]));
        break;
      case 1523: // 3
        gaugeData.TPS = (int)(word(rxmsg.buf[0], rxmsg.buf[1]));
        gaugeData.BATTV = (int)(word(rxmsg.buf[2], rxmsg.buf[3]));
        break;
      case 1524: // 4
        gaugeData.Knock = (int)(word(rxmsg.buf[0], rxmsg.buf[1]));
        gaugeData.EGOc = (int)(word(rxmsg.buf[2], rxmsg.buf[3]));
        break;
      case 1526: // 6
        gaugeData.IAC = (int)(word(rxmsg.buf[6], rxmsg.buf[7])); //IAC = (IAC * 49) / 125;
      case 1529: // 9
        gaugeData.dwell = (int)(word(rxmsg.buf[4], rxmsg.buf[5]));
        break;
      case 1530: // 10
        gaugeData.status1 = rxmsg.buf[0];
        gaugeData.status2 = rxmsg.buf[1];
        gaugeData.status3 = rxmsg.buf[2];
        gaugeData.status6 = rxmsg.buf[6];
        gaugeData.status7 = rxmsg.buf[7];
        break;
      case 1537: // 17
        gaugeData.bstduty = (int)(word(rxmsg.buf[4], rxmsg.buf[5]));
        break;
      case 1548: // 28
        gaugeData.idle_tar = (int)(word(rxmsg.buf[0], rxmsg.buf[1]));
        break;
      case 1551: // 31
        gaugeData.AFR = (int)(word(0x00, rxmsg.buf[0]));
        break;
      case 1574: // 54
        gaugeData.CEL = rxmsg.buf[2];
        break;
      default: 
        // not a broadcast packet
        // assume this is a normal Megasquirt CAN protocol packet and decode the header
        if (rxmsg.ext)
        {
          rxmsg_id.i = rxmsg.id;
          // is this being sent to us?
          if (rxmsg_id.values.to_id == myCANid)
          {
            switch (rxmsg_id.values.msg_type)
            {
            case 1: // MSG_REQ - request data
              // the data required for the MSG_RSP header is packed into the first 3 data bytes
              msg_req_data.bytes.b0 = rxmsg.buf[0];
              msg_req_data.bytes.b1 = rxmsg.buf[1];
              msg_req_data.bytes.b2 = rxmsg.buf[2];
              // Create the tx packet header
              txmsg_id.values.msg_type = 2; // MSG_RSP
              txmsg_id.values.to_id = msCANid; // Megasquirt CAN ID should normally be 0
              txmsg_id.values.from_id = myCANid;
              txmsg_id.values.block = msg_req_data.values.varblk;
              txmsg_id.values.offset = msg_req_data.values.varoffset;
              txmsg.ext = 1;
              txmsg.id = txmsg_id.i;
              txmsg.len = 8;
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
                  txmsg.buf[0] = second();
                  txmsg.buf[1] = minute();
                  txmsg.buf[2] = hour();
                  txmsg.buf[3] = 0;
                  txmsg.buf[4] = day();
                  txmsg.buf[5] = month();
                  txmsg.buf[6] = year() / 256;
                  txmsg.buf[7] = year() % 256;
                  // send the message!
                  Can0.write(txmsg);
                }
              } 
            }
          }
        }
        else
        {
          Serial.write("ID: ");
          Serial.print(rxmsg.id);
        }
    }
  }
}

void divby10(int val)
{
  byte length;

  itoa(val, tempchars, 10);
  length=strlen(tempchars);

  tempchars[length + 1]=tempchars[length]; // null shift right
  tempchars[length]=tempchars[length - 1];
  tempchars[length - 1]='.';
}

// interrupt handler for the encoder button
void ISR_debounce ()
{
  if((long)(millis() - last_millis) >= (DEBOUNCING_TIME * 10))
  {
    display.clearDisplay();;
    if (S_index != 0)
    {
      // deselect brightness
      S_index=0;
      display.clearDisplay();;
      return;
    }
    if (B_index < 4)
    {
      B_index++;
      M_index=0;
      R_index=0;
      myEnc.write(0);
    }
    if (B_index == 4)
    {
      //menu settings
      if (R_index >= 3)
      {
        //save selected - return to main menu
        M_index=0;
        B_index=0;
        R_index=0;
        myEnc.write(0);
        S_index=0;
      }
      if (R_index == 1)
      {
        S_index=1; // select brightness
      }
      if (R_index == 2)
      {
        S_index=2; // select text size, though not implemented
      }
    } // end B_index5
  }
  else
  {
    //end button
    return;
  }
  last_millis = millis();
}

void gauge_histogram()
{
  byte val;

  // 10hz update time
  if (millis() > (validity_window + 80))
  {
    display.clearDisplay();;

    if (R_index > 2 || R_index < 0)
    {
      R_index=0;
      myEnc.write(0);
    }
    switch (R_index)
    {
    // 0-50 value normalization
    case 0:
      val = (gaugeData.AFR - 100) / 2;  // real rough estimation here here of afr on a 0-50 scale
      if (val > 50)
      {
        val=50;
      }
      break;
    case 1:
      val = ((gaugeData.MAP/10) - 30) / 4;
      if (val > 50)
      {
        val = 50;
      }
      break;
    case 2:
      val = (gaugeData.MAT/10) / 4;
      if (val > 50)
      {
        val = 50;
      }
      break;
    }

    histogram_index++;
    if (histogram_index >=64)
    {
      histogram_index=0;
    }
    histogram[histogram_index]=val;

    for (byte i = 0; i < 64; i++)
    {
      int x = histogram_index - i;
      if ( x < 0)
      {
        x = 64 + histogram_index - i;
      }
      display.drawFastVLine((128 - (i * 2)), (64 - histogram[x]), 64, WHITE);
      display.drawFastVLine((127 - (i * 2)), (64 - histogram[x]), 64, WHITE);
    }

    display.setCursor(8,0);
    display.setTextSize(2);
    display.setTextColor(WHITE);

    switch (R_index)
    {
    case 0:
      display.print("AFR ");
      divby10(gaugeData.AFR);
      display.print(tempchars);
      display.drawFastHLine(0, 40, 128, WHITE); // stoich 14.7 line
      for (byte x=1; x < 128; x = x + 2)
      {
        display.drawPixel(x, 40, BLACK);
      }
      break;
    case 1:
      display.print("MAP ");
      display.print(gaugeData.MAP/10);
      display.drawFastHLine(0, 47, 128, WHITE); // Baro line.. roughly 98kpa
      for (byte x=1; x < 128; x = x + 2)
      {
        display.drawPixel(x, 47, BLACK);
      }
      break;
    case 2:
      display.print("MAT ");
      display.print(gaugeData.MAT / 10);
      break;
    }

    validity_window=millis();
  }
}

boolean value_oob()
{
  if (gaugeData.RPM > 100)
  {
    if ((gaugeData.CLT/10) > 260) return 1;
    if (gaugeData.CEL != 0) return 1;
  } 
  else
  {
    return false;
  }

  if ( bitRead(gaugeData.status2,6) == 1)
  {
    return true; // overboost
  }
  return false;
}

void gauge_warning()
{
  byte dlength, llength;
  int midpos;

  display.clearDisplay();;

  if (gaugeData.RPM > 6800)
  {
    dlength=4;
    llength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setTextColor(WHITE);
    display.setCursor(midpos,0);
    display.setTextSize(4);
    display.print(gaugeData.RPM);

    display.setTextSize(2);
    display.setCursor(8, (63 - 15));
    display.print("RPM");

    for (byte i = 0; i < 16; i++)
    {
      leds[i].setRGB(0, 0, 0);
    } // zero out

    byte i = ((gaugeData.RPM - 6800) / 50);

    for (byte p=0; p < i; p++)
    {
      if (i <= 2)
      {
        leds[p+14].setRGB(((255 * neo_brightness) / 16), 0, 0);
      }
      else
      {
        leds[14].setRGB(((255 * neo_brightness) / 16), 0, 0);
        leds[15].setRGB(((255 * neo_brightness) / 16), 0, 0);
        leds[p-2].setRGB(((255 * neo_brightness) / 16), 0, 0);
      }
    }
  }

  if ((gaugeData.CLT/10) > 260)
  {
    dlength=3;
    llength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setTextColor(WHITE);
    display.setCursor(midpos,0);
    display.setTextSize(4);
    display.print(gaugeData.CLT/10);

    display.setTextSize(2);
    display.setCursor(8, (63 - 15));
    display.print("CLT");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(gaugeData.CEL, 0))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("MAP");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(gaugeData.CEL, 1))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("MAT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }

  }

  if (bitRead(gaugeData.CEL, 2))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("CLT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(gaugeData.CEL, 3))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("TPS");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }

  }

  if (bitRead(gaugeData.CEL, 4))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("BATT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(gaugeData.CEL, 5))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("AFR");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(gaugeData.CEL, 6))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("Sync");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(gaugeData.CEL, 7))
  {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("EGT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++)
    {
      neogauge(999, i, 0);
    }
  }

  if ( bitRead(gaugeData.status2,6) == 1)
  {
    gauge_danger();
  }

}

void gauge_debug()
{
  display.clearDisplay();;
  display.setCursor(32,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("m");
  display.print(M_index);
  display.print("r");
  display.print(R_index);
  display.print("s");
  display.print(S_index);
  display.print("b");
  display.println(B_index);

  display.display();
}

void gauge_vitals()
{
  //hard coded for style
  // fonts are 5x7 * textsize
  // size 1 .. 5 x 7
  // size 2 .. 10 x 14
  //Vitals - AFR, RPM, MAP,
  display.clearDisplay();;

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  display.setCursor(41, 0); // 4 char normally - 4 * 10 = 40, - 128 = 88, /2 = 44
  display.println(gaugeData.RPM);
  display.setTextSize(1);
  display.setCursor(21, 7);
  display.print("RPM");

  //line2
  display.setCursor(0, 26);
  display.setTextSize(1);
  display.print("AFR");
  display.setCursor(20, 19);
  display.setTextSize(2);
  divby10(gaugeData.AFR);
  display.print(tempchars);

  display.setCursor(72, 25);
  display.setTextSize(1);
  display.print("CLT");
  display.setCursor(88, 18);
  display.setTextSize(2);
  display.print(gaugeData.CLT/10);

  //line3
  display.setCursor(0, 40);
  display.setTextSize(1);
  display.print("MAP");
  display.setCursor(20, 40);
  display.setTextSize(2);
  display.print(gaugeData.MAP/10);

  // contextual gauge - if idle on, show IAC%
  if ( bitRead(gaugeData.status2,7) == 1)
  {
    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("IAC");
    display.setCursor(92, 40);
    display.setTextSize(2);
    display.print(gaugeData.IAC);
  }
  else if (gaugeData.MAP > gaugeData.Baro)
  {
    int psi;
    display.setCursor(72, 38);
    display.setTextSize(1);
    display.print("PSI");
    display.setCursor(92, 38);
    display.setTextSize(1);
    // 6.895kpa = 1psi
    psi = gaugeData.MAP - gaugeData.Baro;
    psi=(psi * 200) / 1379;
    divby10(psi);
    display.print(tempchars);

    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("MAT");
    display.setCursor(92, 47);
    display.print(gaugeData.MAT/10);

  }
  else
  {
    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("MAT");
    display.setCursor(92,40);
    display.setTextSize(2);
    display.print(gaugeData.MAT/10);
  }

  gauge_bottom();
}

void gauge_bottom()
{
  display.setTextSize(1);
  display.drawFastHLine(1, (63 - 7), 126, WHITE);
  display.setCursor(0, 57);
  display.setTextColor(BLACK, WHITE);

  //CEL
  if ( gaugeData.CEL != 0 )
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
  if ( bitRead(gaugeData.status6,6) == 1)
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
  if ( bitRead(gaugeData.status2,7) == 1)
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
  if ( bitRead(gaugeData.status7,4) == 1)
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
  if ( bitRead(gaugeData.status2,6) == 1)
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

void gauge_single()
{
  byte mult_test;
  char data[10];
  String label;
  byte temp_index;
  display.clearDisplay();

  // if (R_index <= 15)
  // {
    switch (R_index)
    {
      case 0:
        label="RPM";
        itoa(gaugeData.RPM, data, 10);
        break;
      case 1:
        label="AFR";
        divby10(gaugeData.AFR);
        strcpy(data, tempchars);
        break;
      case 2:
        label="Coolant";
        divby10(gaugeData.CLT);
        strcpy(data, tempchars);
        break;
      case 3:
        label="MAP";
        divby10(gaugeData.MAP);
        strcpy(data, tempchars);
        break;
      case 4:
        label="MAT";
        divby10(gaugeData.MAT);
        strcpy(data, tempchars);
        break;
      case 5:
        label="Timing";
        divby10(gaugeData.SPKADV);
        strcpy(data, tempchars);
        break;
      case 6:
        label="Voltage";
        divby10(gaugeData.BATTV);
        strcpy(data, tempchars);
        break;
      case 7:
        label="TPS";
        divby10(gaugeData.TPS);
        strcpy(data, tempchars);
        break;
      case 8:
        label="Knock";
        divby10(gaugeData.Knock);
        strcpy(data, tempchars);
        break;
      case 9:
        label="Barometer";
        divby10(gaugeData.Baro);
        strcpy(data, tempchars);
        break;
      case 10:
        label="EGO Corr";
        divby10(gaugeData.EGOc);
        strcpy(data, tempchars);
        break;
      case 11:
        label="IAC";
        itoa(gaugeData.IAC, data, 10);
        break;
      case 12:
        label="Spark Dwell";
        divby10(gaugeData.dwell);
        strcpy(data, tempchars);
        break;
      case 13:
        label="Boost Duty";
        itoa(gaugeData.bstduty, data, 10);
        break;
      case 14:
        label="Idle Target";
        itoa(gaugeData.idle_tar, data, 10);
        break;
      case 15:
        label="AFR Target";
        divby10(gaugeData.AFR_tar);
        strcpy(data, tempchars);
        break;
    }
  // }
  // TODO: I'm not sure if this is even needed. Need to figure out if this data is useful. If so the data should loop properly. Maybe update the UI for this a bit?
  // else
  // {
  //   temp_index = R_index - 15;
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

  byte dlength=strlen(data);
  byte llength=label.length();
  int midpos;

  // dlength * (width of font) / 2 -1
  // size 2 = 11
  // size 3 = 17
  // size 4 = 23

  midpos = (63 - ((dlength * 23)/ 2));

  display.setTextColor(WHITE);
  display.setCursor(midpos,0);
  display.setTextSize(4);
  display.print(data);

  display.setTextSize(2);
  display.setCursor(8, (63 - 15));
  display.print(label);

  //Additional data for highest/lowest value
  if (R_index == 0)
  {
    if (millis() > (validity_window + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.RPM_highest = gaugeData.RPM;
      validity_window=millis();
    }
    if (gaugeData.RPM > gaugeData.RPM_highest)
    {
      gaugeData.RPM_highest = gaugeData.RPM;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    display.print(gaugeData.RPM_highest);
  }

  if (R_index == 1)
  {
    if (millis() > (validity_window + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.AFR_highest = gaugeData.AFR;
      validity_window=millis();
    }
    if (millis() > (validity_window2 + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.AFR_lowest = gaugeData.AFR;
      validity_window2=millis();
    }
    if (gaugeData.AFR > gaugeData.AFR_highest)
    {
      gaugeData.AFR_highest = gaugeData.AFR;
      validity_window=millis();
    }
    if (gaugeData.AFR < gaugeData.AFR_lowest)
    {
      gaugeData.AFR_lowest = gaugeData.AFR;
      validity_window2=millis();
    }
    display.setTextSize(2);
    display.setCursor(0, 31);
    divby10(gaugeData.AFR_lowest);
    display.print(tempchars);
    display.setCursor((127 - 48), 31);
    divby10(gaugeData.AFR_highest);
    display.print(tempchars);
  }

  if (R_index == 2)
   {
    if (millis() > (validity_window + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.CLT_highest = gaugeData.CLT;
      validity_window=millis();
    }
    if (gaugeData.CLT > gaugeData.CLT_highest)
    {
      gaugeData.CLT_highest = gaugeData.CLT;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 60), 31);
    divby10(gaugeData.CLT_highest);
    display.print(tempchars);
  }

  if (R_index == 3)
  {
    if (millis() > (validity_window + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.MAP_highest = gaugeData.MAP;
      validity_window=millis();
    }
    if (gaugeData.MAP > gaugeData.MAP_highest)
    {
      gaugeData.MAP_highest = gaugeData.MAP;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    divby10(gaugeData.MAP_highest);
    display.print(tempchars);
  }

  if (R_index == 4)
  {
    if (millis() > (validity_window + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.MAT_highest = gaugeData.MAT;
      validity_window=millis();
    }
    if (gaugeData.MAT > gaugeData.MAT_highest)
    {
      gaugeData.MAT_highest = gaugeData.MAT;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    divby10(gaugeData.MAT_highest);
    display.print(tempchars);
  }

  if (R_index == 8)
  {
    if (millis() > (validity_window + 30000))
    {
      // Highest value resets after 30 seconds
      gaugeData.Knock_highest = gaugeData.Knock;
      validity_window=millis();
    }
    if (gaugeData.Knock > gaugeData.Knock_highest)
    {
      gaugeData.Knock_highest = gaugeData.Knock;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    divby10(gaugeData.Knock_highest);
    display.print(tempchars);
  }
  display.display();
}

void gauge_menu()
{
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);

  if (S_index == 0) // Nothing selected
  {
    if (R_index > 3)
    {
      R_index = 3;
    }
    switch (R_index)
    {
      case 0:
        //line1
        display.setTextColor(BLACK, WHITE);
        display.println("_Menu");
        //line2
        display.setTextColor(WHITE);
        display.print("Lum: ");
        display.println(neo_brightness);
        //line3
        display.print("Text: ");
        display.println(g_textsize);
        //line4
        display.print("Save");
        display.display();
        break;

      case 1: // Brightness highlighted
        //line1
        display.setTextColor(WHITE);
        display.println("_Menu");
        //line2
        display.setTextColor(BLACK, WHITE);
        display.print("Lum: ");
        display.println(neo_brightness);
        //line3
        display.setTextColor(WHITE);
        display.print("Text: ");
        display.println(g_textsize);
        //line4
        display.print("Save");
        display.display();
        break;

      case 2: // Text size highlighted
        //line1
        display.setTextColor(WHITE);
        display.println("_Menu");
        //line2
        display.print("Lum: ");
        display.println(neo_brightness);
        //line3
        display.setTextColor(BLACK, WHITE);
        display.print("Text: ");
        display.println(g_textsize);
        //line4
        display.setTextColor(WHITE);
        display.print("Save");
        display.display();
        break;

      case 3: // Save highlighted
        //line1
        display.setTextColor(WHITE);
        display.println("_Menu");
        //line2
        display.print("Lum: ");
        display.println(neo_brightness);
        //line3
        display.setTextColor(WHITE);
        display.print("Text: ");
        display.println(g_textsize);
        //line4
        display.setTextColor(BLACK, WHITE);
        display.print("Save");
        display.display();
        break;
    } // end switch
  } // end Nothing selected

  if (S_index == 1) // Brightness Selected
  {
    neo_brightness=R_index;
    display.clearDisplay();;
    display.setCursor(0,0);
    if (R_index > 8)
    {
      R_index = 8;
      myEnc.write(8);
      neo_brightness = 8;
    }
    if (R_index < 1)
    {
      R_index = 1;
      myEnc.write(1);
      neo_brightness=1;
    }

    //line1
    display.setTextColor(WHITE);
    display.println("_Menu");
    //line2
    display.setTextColor(BLACK, WHITE);
    display.print("Lum: ");
    display.println(neo_brightness);
    //line3
    display.setTextColor(WHITE);
    display.print("Text: ");
    display.println(g_textsize);
    //line4
    display.print("Save");
    display.display();
  }// Brightness selected

  if (S_index == 2)
  {
    // temp=M_index;
    g_textsize=R_index;
    if (R_index > 4)
    {
      R_index = 4;
      myEnc.write(4);
      g_textsize = 4;
    }
    if (R_index < 1)
    {
      R_index = 1;
      myEnc.write(1);
      g_textsize=1;
    }

    //line1
    display.setTextColor(WHITE);
    display.println("_Menu");
    //line2
    display.print("Lum: ");
    display.println(neo_brightness);
    //line3
    display.setTextColor(BLACK, WHITE);
    display.print("Text: ");
    display.println(g_textsize);
    //line4
    display.setTextColor(WHITE);
    display.print("Save");
    display.display();
  } // Text size selected
} // end gauge_menu

void gauge_danger()
{
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();;
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

void neogauge(int val, byte led, byte enable_warning)
{
  unsigned int red, green, blue;
  val = val/2;

  if ( val > 500 )
  {
    if (enable_warning > 0)
    {
      leds[led].setRGB(0, 0, 0);
      FastLED.show();
      delay(50);
      red = (255 * neo_brightness) / 16;
      leds[led].setRGB(red, 0, 0);
      FastLED.show();
    }
    else
    {
      red = (255 * neo_brightness) / 16;
      leds[led].setRGB(red, 0, 0);
      FastLED.show();
    }
  }
  else if ( val < 0 )
  {
    if (enable_warning > 0)
    {
      leds[led].setRGB(0, 0, 0);
      FastLED.show();
      delay(50);
      blue = (255 * neo_brightness) / 16;
      leds[led].setRGB(0, 0, blue);
      FastLED.show();
    }
    else
    {
      blue = (255 * neo_brightness) / 16;
      leds[led].setRGB(0, 0, blue);
      FastLED.show();
    }
  }
  else if ((val >= 0) && (val <= 500))
  {
    red =   pgm_read_byte (&ledarray[val].r0);
    green = pgm_read_byte (&ledarray[val].g0);
    blue =  pgm_read_byte (&ledarray[val].b0);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led].setRGB(red, green, blue);
    // FastLED.show();
  }
}

void neogauge4led(int val, byte led0, byte led1, byte led2, byte led3, byte enable_warning)
{
  unsigned int red, green, blue;
  val = val/2;

  if ( val > 500 )
  {
    if (enable_warning > 0 && !gaugeBlink)
    {
      leds[led0].setRGB(0, 0, 0);
      leds[led1].setRGB(0, 0, 0);
      leds[led2].setRGB(0, 0, 0);
      leds[led3].setRGB(0, 0, 0);
    }
    else
    {
      red = (255 * neo_brightness) / 16;
      leds[led0].setRGB(red, 0, 0);
      leds[led1].setRGB(red, 0, 0);
      leds[led2].setRGB(red, 0, 0);
      leds[led3].setRGB(red, 0, 0);
    }

  }
  else if ( val < 0 )
  {
    if (enable_warning > 0 && !gaugeBlink)
    {
      leds[led0].setRGB(0, 0, 0);
      leds[led1].setRGB(0, 0, 0);
      leds[led2].setRGB(0, 0, 0);
      leds[led3].setRGB(0, 0, 0);
    }
    else
    {
      blue = (255 * neo_brightness) / 16;
      leds[led0].setRGB(0, 0, blue);
      leds[led1].setRGB(0, 0, blue);
      leds[led2].setRGB(0, 0, blue);
      leds[led3].setRGB(0, 0, blue);
    }
  }
  else
  {
    red   = pgm_read_byte (&ledarray[(val)].r0);
    green = pgm_read_byte (&ledarray[(val)].g0);
    blue  = pgm_read_byte (&ledarray[(val)].b0);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led0].setRGB(red, green, blue);

    red   = pgm_read_byte (&ledarray[(val)].r1);
    green = pgm_read_byte (&ledarray[(val)].g1);
    blue  = pgm_read_byte (&ledarray[(val)].b1);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led1].setRGB(red, green, blue);

    red   = pgm_read_byte (&ledarray[(val)].r2);
    green = pgm_read_byte (&ledarray[(val)].g2);
    blue  = pgm_read_byte (&ledarray[(val)].b2);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led2].setRGB(red, green, blue);

    red   = pgm_read_byte (&ledarray[(val)].r3);
    green = pgm_read_byte (&ledarray[(val)].g3);
    blue  = pgm_read_byte (&ledarray[(val)].b3);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led3].setRGB(red, green, blue);
  }
}

void write_neopixel()
{
  long temp;

  temp = (gaugeData.RPM * 1000) / REVLIMIT;
  neogauge4led(temp, 1, 0, 15, 14, 1); // RPM min 0 max REVLIMIT

  temp = ((gaugeData.AFR * 2) * 100) / 59;
  if (gaugeData.AFR <= 147)
  {
    temp = (pow((gaugeData.AFR - 147),3) / 150) + 500;
  }
  else if (gaugeData.AFR > 147)
  {
    temp = (pow((gaugeData.AFR - 147),3) / 20) + 500;
  }

  neogauge4led(temp, 9, 10, 11, 12, 0); // AFR

  temp=gaugeData.TPS;
  neogauge(temp, 2, 0); //TPS - min 0 max 1000

  temp=(gaugeData.CLT * 5) / 12; //CLT - min ? mid 120 max 240
  neogauge(temp, 4, 1);


  temp=(gaugeData.MAT * 5) / 7; //MAT - min ? mid 70 max 140
  neogauge(temp, 6, 0);

  temp=gaugeData.MAP/2;
  neogauge(temp, 8, 0); //MAP - min impossible mid 100kpa max 200kpa

  // will need to play with this some, 50 looks reasonable though
  temp=((gaugeData.AFR - gaugeData.AFR_tar) * 50) + 500;
  neogauge(temp, 13, 0);

  leds[3].setRGB(0, 0, 0); // unallocated
  leds[5].setRGB(0, 0, 0);
  leds[7].setRGB(0, 0, 0);

  FastLED.show();

//todo: oil temp, oil pressure, EGT
//todo: rearrange LED's into something nicer
//todo: might be faster to do a final FastLED.show here instead of inside the neogauge functions
}
