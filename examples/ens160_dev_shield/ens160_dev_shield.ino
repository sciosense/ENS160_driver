/***************************************************************************
  ENS160 Development Shield rev 3.0

  This is an example for operating the develeopment shield including
  - ENS160 for air quality sensing
  - ENS210 for temperature & humidity sensing
  - Neopixel RGBW LED for visual feedback

  Written for Sciosense / 29-July-2021
 ***************************************************************************/

#include <Wire.h>
int ArduinoLED = 13;

//-------------------------------------------------------------
//Neopixel related items
//-------------------------------------------------------------

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN     12
#define LED_COUNT   1
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint16_t new_TVOC;
uint16_t new_eCO2;
uint16_t colorRed;
uint16_t colorGreen;
uint16_t colorBlue;

//-------------------------------------------------------------
//ENS160 related items
//-------------------------------------------------------------
#include "ScioSense_ENS160.h"  // ENS160 library
ScioSense_ENS160      ens160;

//-------------------------------------------------------------
//ENS210 related items
//-------------------------------------------------------------
#include "ScioSense_ENS210.h"  // ENS160 library
ScioSense_ENS210      ens210;

void setup() {
  //-------------------------------------------------------------
  // General setup steps
  //-------------------------------------------------------------

  Serial.begin(115200);

  while (!Serial) {}

  //Switch on LED for init
  pinMode(ArduinoLED, OUTPUT);
  digitalWrite(ArduinoLED, LOW);

  Serial.println("------------------------------------------------------------");
  Serial.println("ENS160 Development Shield rev 3.0");
  Serial.println();
  Serial.println("Sensor readout & RGBW indication");
  Serial.println();
  Serial.println("------------------------------------------------------------");
  delay(1000);

  //-------------------------------------------------------------
  // Neopixel related items
  //-------------------------------------------------------------
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  //-------------------------------------------------------------
  //ENS160 related items
  //-------------------------------------------------------------
  Serial.print("ENS160...");
  
  ens160.begin(); Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available()) {
    // Print ENS160 versions
    Serial.print("\t Rev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());

    Serial.print("\t Standard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
  }

  //-------------------------------------------------------------
  //ENS210 related items
  //-------------------------------------------------------------
  Serial.print("ENS210...");
  ens210.begin();
  Serial.println(ens210.available() ? "done." : "failed!");
  ens210.setSingleMode(false); 

}

/*--------------------------------------------------------------------------
  MAIN LOOP FUNCTION
  Cylce every 1000ms, perform measurement and adjust RGBW output
 --------------------------------------------------------------------------*/

void loop() {
  ens210.measure();
  ens160.measure();

  new_TVOC = (uint16_t)ens160.getTVOC();
  if (new_TVOC < 500) new_TVOC = 0; 
  else if (new_TVOC > 5000) new_TVOC = 5000;
  else new_TVOC = new_TVOC - 500;

  new_eCO2 = (uint16_t)ens160.geteCO2();
  if (new_eCO2 < 400) new_eCO2 = 400;
  else if (new_eCO2 > 5000) new_eCO2 = 5000;

  colorRed = max(map(new_eCO2,400,5000,0,254), map(new_TVOC,0,5000,0,254));
  colorGreen = 254 - colorRed;
  colorBlue = 0;
  
  strip.setPixelColor(0, strip.Color(colorRed, colorGreen, colorBlue));
  strip.show();

  Serial.print("AQI (0x21): ");Serial.print(ens160.getAQI());Serial.print("\t");
  Serial.print("TVOC (0x22): ");Serial.print(new_TVOC);Serial.print("ppb\t");
  Serial.print("eCO2 (0x24): ");Serial.print(new_eCO2);Serial.print("ppm\t");
  Serial.print("R HP0: ");Serial.print(ens160.getHP0());Serial.print("Ohm\t");
  Serial.print("R HP1: ");Serial.print(ens160.getHP1());Serial.print("Ohm\t");
  Serial.print("R HP2: ");Serial.print(ens160.getHP2());Serial.print("Ohm\t");
  Serial.print("R HP3: ");Serial.print(ens160.getHP3());Serial.print("Ohm\t");
  Serial.print("MISR: ");Serial.print(ens160.getMISR());Serial.print("\t");
  Serial.print("T: ");Serial.print(ens210.getTempCelsius());Serial.print("°C\t");
  Serial.print("rH: "); Serial.print(ens210.getHumidityPercent());Serial.println("%");

  delay(1000);
}
