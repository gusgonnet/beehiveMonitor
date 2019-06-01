/*
 * Project beehiveMonitor
 * Description: monitor the state of beehives
 * Author: Gustavo Gonnet gusgonnet@gmail.com
 * Date: May 26th 2019
 */

/*******************************************************************************
PINOUT
D9 - DS18B20
*******************************************************************************/

// #include "lib/elapsedMillis/elapsedMillis.h"
#include "AnalogSmooth.h"
#include "Adafruit_ADT7410.h"
#include "Adafruit_ADXL343.h"

#define APP_NAME "beehiveMonitor"
#define VERSION "Version 0.01"

#define ALWAYS_ONLINE
#define USE_ADT7410
#define USE_DS18B20
#define DS18B20_PIN D9

/*******************************************************************************
 * changes in version 0.01:
       * Particle Build link: https://go.particle.io/shared_apps/5a6b8e6d7e1a22f899001161
       * Initial version
*******************************************************************************/

//enable the user code (our program below) to run in parallel with cloud connectivity code
// source: https://docs.particle.io/reference/firmware/photon/#system-thread
SYSTEM_THREAD(ENABLED);

#if PLATFORM_ID == PLATFORM_BORON
FuelGauge batteryMonitor;
// threshold in percentage
#define CRITICAL_BATTERY_BORON 20
#endif

#if PLATFORM_ID == PLATFORM_XENON
// https://docs.particle.io/reference/device-os/firmware/xenon/#battery-voltage
AnalogSmooth analogSmoothBATT = AnalogSmooth(20); // get 20 samples

// https://community.particle.io/t/can-argon-or-xenon-read-the-battery-state/45554/35?u=gusgonnet
// threshold  in volts
#define CRITICAL_BATTERY_XENON 3.4
#endif

// these variables are set in setup()
// they are easier to use in some parts of the code
// rather than using the define USE_DS18B20, USE_ADT7410, ALWAYS_ONLINE, etc
bool use_ds18b20 = false;
bool use_adt7410 = false;
bool alwaysOnline = false;
bool boron = false;
bool xenon = false;

float temp_ADT7410 = 0;
float batteryReading = 0;

/*******************************************************************************
vvvv    DS18B20 related    vvvv
*******************************************************************************/
bool useFahrenheit = true;

#ifdef USE_DS18B20
#include <DS18B20.h>
#include <math.h>
// Sets Pin DS18B20_PIN for DS18B20 Temp Sensor
// true: this is the only sensor on bus
DS18B20 ds18b20(DS18B20_PIN, true);
#endif

// let's increase the MAXRETRY as per this advise:
// https://community.particle.io/t/ds18b20-temperature-sensor-and-correct-library-for-boron/46933/5?u=gusgonnet
const int MAXRETRY = 30;

double temp_DS18B20_celsius;
double temp_DS18B20_fahrenheit;

/*******************************************************************************
^^^^    DS18B20 related    ^^^^
*******************************************************************************/

bool publishStatusFlag = false;
bool movementDetected = false;

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{

  Particle.function("forcePublishStatus", forcePublishStatus);

#if PLATFORM_ID == PLATFORM_BORON
  boron = true;
  PMIC pmic;                               //Initalize the PMIC class so you can call the Power Management functions below.
  pmic.setInputVoltageLimit(5080);         //  for 6V Solar Panels
  pmic.setInputCurrentLimit(2000);         // 2000 mA, higher than req'd
  pmic.setChargeVoltage(4208);             //  Set Li-Po charge termination voltage to 4.21V,  Monitor the Enclosure Temps
  pmic.setChargeCurrent(0, 0, 1, 1, 1, 0); // 1408 mA [0+0+512mA+256mA+128mA+0] + 512 Offset
  pmic.enableDPDM();
#endif

#if PLATFORM_ID == PLATFORM_XENON
  xenon = true;
  pinMode(BATT, INPUT);
#endif

// these variables are easier to use in some parts of the code
// rather than using the define USE_DS18B20, USE_ADT7410, ALWAYS_ONLINE, etc
#ifdef ALWAYS_ONLINE
  alwaysOnline = true;
#endif
#ifdef USE_DS18B20
  use_ds18b20 = true;
#endif
#ifdef USE_ADT7410
  use_adt7410 = true;
#endif

  Serial.begin(115200);
}

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
void loop()
{
  // check if the forcePublishStatus cloud function was called
  checkPublishStatusFlag();

#if PLATFORM_ID == PLATFORM_XENON
  smoothBATT();
#endif
}

/*******************************************************************************
 * Function Name  : publishStatus
 * Description    : publish status message
 *******************************************************************************/
void publishStatus()
{

  //refresh readings
  getTempDS18B20();

#define LITTLE 50
  char tempChar[LITTLE] = "";

#define BUFFER 623
  char pubChar[BUFFER] = "";
  snprintf(pubChar, BUFFER, "Movement: %d", movementDetected);

  if (boron)
  {
#if PLATFORM_ID == PLATFORM_BORON
    snprintf(tempChar, LITTLE, ", SoC: %.2f%%", batteryMonitor.getSoC());
#endif
    strcat(pubChar, tempChar);
  }

  if (xenon)
  {
    snprintf(tempChar, LITTLE, ", Battery: %.2f Volts", batteryReading);
    strcat(pubChar, tempChar);
  }

  if (use_adt7410)
  {
    snprintf(tempChar, LITTLE, ", adt7410: %.2f", temp_ADT7410);
    strcat(pubChar, tempChar);
  }

  if (use_ds18b20)
  {
    if (useFahrenheit)
    {
      snprintf(tempChar, LITTLE, ", ds18b20: %.2f", temp_DS18B20_fahrenheit);
    }
    else
    {
      snprintf(tempChar, LITTLE, ", ds18b20: %.2f", temp_DS18B20_celsius);
    }
    strcat(pubChar, tempChar);
  }

  Particle.publish("STATUS", pubChar, PRIVATE | WITH_ACK);
}

/*******************************************************************************
 * Function Name  : checkPublishStatusFlag
 * Description    : call publish status message if the publishStatus() cloud function was called
 *******************************************************************************/
void checkPublishStatusFlag()
{
  if (publishStatusFlag)
  {
    publishStatusFlag = false;
    publishStatus();
  }
}

/*******************************************************************************
 * Function Name  : forcePublishStatus
 * Description    : cloud function to publish the status of the system.
                    Since it's bad practice to publish in a cloud function,
                    we set a flag and read the flag from the loop() function
 *******************************************************************************/
int forcePublishStatus(String dummy)
{
  publishStatusFlag = true;
  return 0;
}

/*******************************************************************************
 * Function Name  : smoothBAT
 * Description    : read and smooth the BATT pin
 *******************************************************************************/
void smoothBATT()
{
#if PLATFORM_ID == PLATFORM_XENON
  // source: https://github.com/kennethlimcp/particle-examples/blob/master/vbatt-argon-boron/vbatt-argon-boron.ino
  float analog = analogRead(BATT) * 0.0011224;
  batteryReading = analogSmoothBATT.smooth(analog);
#endif
}

/*******************************************************************************
 * Function Name  : getTempDS18B20
 * Description    : get reading from the DS18b20 sensor
 *******************************************************************************/
void getTempDS18B20()
{
  float _temp;
  int i = 0;

  do
  {
    _temp = ds18b20.getTemperature();
    Serial.println(i);
    Serial.println(_temp);
    Serial.println(ds18b20.crcCheck());

  } while (((_temp == NAN) || !ds18b20.crcCheck()) && MAXRETRY > i++);

  if (i < MAXRETRY)
  {
    temp_DS18B20_celsius = _temp;
    temp_DS18B20_fahrenheit = ds18b20.convertToFahrenheit(_temp);
  }
  else
  {
    temp_DS18B20_celsius = temp_DS18B20_fahrenheit = NAN;
    Serial.println("Invalid reading");
  }

#define LITTLE 50
  char tempChar[LITTLE] = "";
  snprintf(tempChar, LITTLE, "ds18b20 fahrenheit: %.2f", temp_DS18B20_fahrenheit);
  Serial.println(tempChar);

  snprintf(tempChar, LITTLE, "ds18b20 celsius: %.2f", temp_DS18B20_celsius);
  Serial.println(tempChar);
}
