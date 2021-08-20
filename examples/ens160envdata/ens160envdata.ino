/***************************************************************************
  ENS160 - Digital Air Quality Sensor
  
  This is an example for ENS160 basic reading in standard mode
  In addition temperatuer & humidity sensor data from ENS210 is used for compensation 
    
  Updated by Sciosense / 29-July-2021
 ***************************************************************************/

#include <Wire.h>
int ArduinoLED = 13;

//-------------------------------------------------------------
//ENS160 related items
//-------------------------------------------------------------
#include "ScioSense_ENS160.h"  // ENS160 library
ScioSense_ENS160      ens160(ENS160_I2CADDR_0);
//ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

//-------------------------------------------------------------
//ENS210 related items
//-------------------------------------------------------------
#include "ScioSense_ENS210.h"
ScioSense_ENS210      ens210;

/*--------------------------------------------------------------------------
  SETUP function
  initiate sensors
 --------------------------------------------------------------------------*/
void setup() {

  //-------------------------------------------------------------
  // General setup steps
  //-------------------------------------------------------------

  Serial.begin(11520);

  while (!Serial) {}

  //Switch on LED for init
  pinMode(ArduinoLED, OUTPUT);
  digitalWrite(ArduinoLED, LOW);

  Serial.println("------------------------------------------------------------");
  Serial.println("ENS160 - Digital air quality sensor");
  Serial.println();
  Serial.println("Sensor readout with rH & T compensation");
  Serial.println();
  Serial.println("------------------------------------------------------------");

  delay(1000);

  //-------------------------------------------------------------
  // ENS160 related items
  //-------------------------------------------------------------
  Serial.print("ENS160...");
  ens160.begin();
  Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available()) {
    // Print ENS160 versions
    Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());

    Serial.print("\tStandard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!" );
  }
  
  //-------------------------------------------------------------
  // ENS210 related items
  //-------------------------------------------------------------
  Serial.print("ENS210...");
  ens210.begin();
  Serial.println(ens210.available() ? "done." : "failed!");
  ens210.setSingleMode(false);

  //blink LED
  digitalWrite(ArduinoLED, HIGH);
  delay(100);
  digitalWrite(ArduinoLED, LOW);
}

/*--------------------------------------------------------------------------
  MAIN LOOP FUNCTION
  Cylce every 1000ms and perform measurement
 --------------------------------------------------------------------------*/

void loop() {
 
  if (ens160.available()) {
    if (ens210.available()) {
      ens210.measure();
      ens160.set_envdata210(ens210.getDataT(),ens210.getDataH());
    }
    ens160.measure();
  }
  
  Serial.print("AQI: ");Serial.print(ens160.getAQI());Serial.print("\t");
  Serial.print("TVOC: ");Serial.print(ens160.getTVOC());Serial.print("ppb\t");
  Serial.print("eCO2: ");Serial.print(ens160.geteCO2());Serial.print("ppm\t");
  Serial.print("R HP0: ");Serial.print(ens160.getHP0());Serial.print("Ohm\t");
  Serial.print("R HP1: ");Serial.print(ens160.getHP1());Serial.print("Ohm\t");
  Serial.print("R HP2: ");Serial.print(ens160.getHP2());Serial.print("Ohm\t");
  Serial.print("R HP3: ");Serial.print(ens160.getHP3());Serial.print("Ohm\t");
  
  Serial.print("Temperature: ");Serial.print(ens210.getTempCelsius());Serial.print("°C\t");
  Serial.print("Humidity: "); Serial.print(ens210.getHumidityPercent());Serial.print("%");
    Serial.println();

  delay(1000);
}
