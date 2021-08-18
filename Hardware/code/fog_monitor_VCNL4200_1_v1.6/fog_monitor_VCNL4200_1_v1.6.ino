/*
  V1.6 based on V1.5
  adds standard deviation output for VCNLext positions 4,5, and 6
  serial output is commented out
  VCNL4 is used for indicator LED

  V1.5 based on V1.3
  sets sunlight protection parameters for individual sensors
  currenly sunlight protection is enabled for all sensors
  currently "2X sunlight immunity" and "1.5X sun protect" are NOT activated

  V1.3 Polls sensors sequentially to avoid interference
  logfile name is 19FOG**.csv
  also changed PS_SP (in PS_MS) from 1 1.5X sun protect to default

  V1.2 Uses forced sampling of VCNL4200 at 1 Hz
  all data read at 1 Hz
  also changed to SdFat.h library for microSD card

  v1.1 changes data VCNL data acquisition rate to 2 Hz
  also reduces sunlight immunity from "2x typical sunlight immunity" to "typical sunlight immunity"

  v1.0 code for data acquistion from multiple VCNL4200 (ambient light and IR proximity)
  I2C multiplexer used to talk to multiple sensors with same address
  starting point is "fog_monitor_VCNL4200_MAX30105_v1.2.ino"
  records data to disk once per minute using RTC for timing
  logfile name is 18FOG**.csv

  uses Statistic to calculate averages and standard deviations, used F() for all strings to reduce memory use
  Includes ambient light (VCNL4200) measurements (1 Hz)

  This program reads data from the Vishay VCNL4200 IR Proximity and Ambient Light Sensor
  Backscatter of IR light is used to determine fog intensity
  1s averages are output to the serial monitor at 9600 baud

  VCNL4020
  IC Supply voltage = 3.3 V (board includes a voltage regulator from 5V to 3.3V)
  External LED supply voltage = 5 V
  Readings are obtained at 3Hz
*/

#include <Wire.h> // Standard I2C Library
#include <SPI.h>
#include "Statistic.h" //statistics library

Statistic VCNL1proxStats;

Statistic VCNL1ambStats;

// General variables ////////////////////////////////////////////
int bytes;
int reg;
char data;
unsigned long PreviousMillis = 0;
unsigned long OneHzInterval = 60000; // Interval for averaging data if millis() is used (in milliseconds)
int previousMinute = 0;
int longIntervalRTC = 1; // Interval for averaging data if real time clock is used (in minutes)

// VCNL4200 Proximity variables (1 Hz) ////////////////////////////////////////////
char VCNLaddress = 0x51; // VCNL4200 address

int VCNLcount = 0;

uint16_t VCNLprox[8] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned long VCNLshortPreviousMillis = 0;
unsigned long VCNLshortInterval = 1000;

// VCNL4200 Ambient Light variables (1 Hz)  //////////////////////////////////

uint16_t VCNLamb[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//INITIALIZATION
void setup()
{
  Wire.begin(); // Starts the I2C Connection
  Serial.begin(9600); // Starts the Serial Communication
  pinMode(5, OUTPUT);  // use LED on pin 5 to indicate errors (blink) or success (steady)

  VCNL1proxStats.clear();

  VCNL1ambStats.clear();

  //  SD card setup  ///////////////////////////////////////////////////////////////////////////////////////////
  // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  //******************************************
  // set date time callback function
  //SdFile::dateTimeCallback(dateTime);
  //********************************************

  //Serial.print(F("Logging to: "));
  //Serial.println(filename);

  // VCNL4200 Setup ////////////////////////////////////////////////////////////////////////////////////////////////
  // Check each VCNL4200 ID and print it to serial monitor

  Wire.beginTransmission(VCNLaddress);
  Wire.write(0x0E);  // Register #0E(low) (ID_L) Product ID Revision Register, Should be 58(hex)
  Wire.endTransmission(false);
  Wire.requestFrom(VCNLaddress, 2);

  // /Read VCNL4200 Proximity Data
  byte VCNLdevIDL = Wire.read();
  byte VCNLdevIDH = Wire.read();

  Serial.print(F("VCNL Device ID: "));
  Serial.println(VCNLdevIDL, HEX);


  // configure VCNL4200 proximity sensor
  // configure VCNL4200 register 3 (PS_CONF1 and PS_CONF2)
  Wire.beginTransmission(VCNLaddress);
  Wire.write(0x03); // low byte = PS_CONF1, high byte = PS_CONF2
  Wire.write(0b11001010); //PS_CONF1  [7:6] PS_DUTY (1/1280 = 1:1)   [5:4] PS_PERS   [3:1] PS_IT pulse width (1:0:1 = 9T)   [0] PS_SD (0 = power on)
  Wire.write(0b00001000); //PS_CONF2    [7:4] reserved     [3] PS_HD (1 = 16 bit)     [2] reserved      [1:0]  PS_INT  (0:0 interupt disable)
  Wire.endTransmission();

  // configure VCNL4200 register 4 (PS_CONF3 and PS_MS)   set PS_AF to active force mode
  Wire.beginTransmission(VCNLaddress);
  Wire.write(0x04); // low byte = PS_CONF3, high byte = PS_MS
  Wire.write(0b01101001); //PS_CONF3  [7] reserved [6:5] PS_MPS (1:1 8 pulses) [4] PS_SMART_PERS (0 disable) [3] PS_AF (0 active force mode disable) [2] PS-TRIG (0) [1] PS_SC_ADV (1 2Xsunlight immunity) [0] PS_SC_EN (1 sunlight cancellation enable)
  Wire.write(0b00000111); //PS_MS    [7:6] reserved [5] PS_MS (0 prox normal operation with interrupt) [4] PS_SP (1 1.5X sun protect) [3] PS_SPO (1 sun protect mode output FFh) [2:0] LED_I (1:1:1 200 mA)
  Wire.endTransmission();

  // configure VCNL4200 ambient light sensor
  // configure VCNL4200 register 0 (ALS_CONF)
  Wire.beginTransmission(VCNLaddress);
  Wire.write(0x00); // low byte = ALS_CONF, high byte = reserved
  Wire.write(0b00000000); //ALS_CONF  [7:6] ALS_IT time integration (0:0 50ms) [5] ALS_INT_SWITCH (0) [4] reserved [3:2] ALS_PERS (0) [1] ALS_INT_EN (0 interupt disable) [0] ALS_SC (0 ALS power on)
  Wire.endTransmission();


  //  Serial.println(F("VCNL1,VCNL2,VCNL3,VCNL4,VCNL5,VCNL6,AMB1,AMB2,AMB3,AMB4,AMB5,AMB6,VCNLcount,OneHzCount"));  //data file header
  //Serial.println(F("Date/Time,VCNL1,AMB1,AMB2,AMB3,AMB4,AMB5,AMB6,VCNLcount,VCNLstdev,VCNL2stdev,VCNL3stdev,VCNL4stdev,VCNL5stdev,VCNL6stdev"));  //data file header
  //Serial.println(F("VCNL1prox,VCNL2prox,VCNL1amb,VCNL2amb,VCNLcount,OneHzCount"));  //data file header
  //logfile.println(F("Date/Time,VCNL1prox,VCNL2prox,VCNL1amb,VCNL2amb,VCNLcount,OneHzCount"));  //data file header
  delay(1000);
}



// MAIN
void loop() // This function will run forever after initialization
{

  // Read VCNL4200 Proximity Data ////////////////////////////////////////////////////////////////////////////////////////
  unsigned long currentMillis = millis();
  if (currentMillis - VCNLshortPreviousMillis >= VCNLshortInterval) {
    VCNLshortPreviousMillis = currentMillis;

    //Request one proximity measurement by setting PS-TRIG = 1
    //Turn sunlight cancelation off for sensors at positions 3 and 4 (one internal IRED and one external IRED)

    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x04); // low byte = PS_CONF3, high byte = PS_MS
    Wire.write(0b01101101); //PS_CONF3  [7] reserved [6:5] PS_MPS (1:1 8 pulses) [4] PS_SMART_PERS (0 disable) [3] PS_AF (1 active force mode enable) [2] PS-TRIG (1) [1] PS_SC_ADV (1 2Xsunlight immunity) [0] PS_SC_EN (1 sunlight cancellation enable)
    Wire.write(0b00000111); //PS_MS    [7:6] reserved [5] PS_MS (0 prox normal operation with interrupt) [4] PS_SP (1 1.5X sun protect) [3] PS_SPO (1 sun protect mode output FFh) [2:0] LED_I (1:1:1 200 mA)
    Wire.endTransmission();

    Wire.beginTransmission(VCNLaddress);
    Wire.write(0x08);
    Wire.endTransmission(false);
    Wire.requestFrom(VCNLaddress, 2);

    // /Read VCNL4200 Proximity Data
    uint8_t LSB = Wire.read();
    uint16_t MSB = Wire.read();

    VCNLprox[2] = (MSB <<= 8) + LSB;
    delay(30);

    VCNL1proxStats.add(VCNLprox[2]);

    // Read VCNL4200 ambient data /////////////////////////////////////////////////////////////////////
    currentMillis = millis();
    if (currentMillis - PreviousMillis >= OneHzInterval) {
      PreviousMillis = currentMillis;

      Wire.beginTransmission(VCNLaddress);
      Wire.write(0x09);
      Wire.endTransmission(false);
      Wire.requestFrom(VCNLaddress, 2);

      // /Read VCNL4200 ALS Data
      uint8_t LSB = Wire.read();
      uint16_t MSB = Wire.read();

      VCNLamb[2] = (MSB <<= 8) + LSB;

      VCNL1ambStats.add(VCNLamb[2]);
      //   OneHzCount++;
    }

    // Output to serial

    // Output FCNL4200 data /////////////////////
    Serial.print(F(" VCNL1prox: "));
    Serial.print(VCNL1proxStats.average(), 0);
    Serial.print(F("       "));
    Serial.print(F(" VCNL1amb:  "));
    Serial.print(VCNL1ambStats.average(), 0);


    Serial.print(F("       VCNLCount: "));
    Serial.println(VCNL1proxStats.count());

    //   Serial.print(F(" OneHzCount:  "));
    //   Serial.println(OneHzCount);


    // Reset sum and count values

    // OneHzCount=0;


    VCNL1proxStats.clear();

    VCNL1ambStats.clear();

  }

}

byte i2c_read(char address, int reg, int bytes)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, bytes);
  return data = Wire.read();
}

void i2c_write(char address, int reg, char data)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
