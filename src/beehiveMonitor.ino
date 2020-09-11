/*
 * Project beehiveMonitor
 * Description: monitor the state of beehives
 * Author: Gustavo Gonnet gusgonnet@gmail.com
 * Date: May 26th 2019
 */

/*******************************************************************************
PINOUT DS18B20
D9 - DS18B20

********************************************************************************
PINOUT ADT7410 https://learn.adafruit.com/adt7410-breakout/pinouts

VIN: This is the voltage input to power for the sensor. 
     You can connect either 5V or 3.3V to this, depending on the logic level of the MCU you are using. (Do not exceed 5V on this pin or you will permanently damage the sensor!)
GND: Connect the GND pin on the breakout to a GND pin on your MCU to have a common reference point.

SCL: This is the I2C clock line, which is pulled high to the same logic level as the VIN pin. 
     Connect this to SCL on your development board.
SDA: This is the I2C data line, which is pulled high to the same logic level as the VIN pin. 
     Connect this to SDA on your development board.

The I2C address on the ADT7410 will default to 0X48.

********************************************************************************
PINOUT ADXL343 https://learn.adafruit.com/adxl343-breakout-learning-guide/pinout

VIN - This is the input to the 3.3V voltage regulator, which makes it possible to use the 3.3V sensor on 5V systems. 
      It also determines the logic level of the SCL and SDA pins. Connect this to 3.3V on the MCU for 3.3V boards (Adafruit Feathers), or 5.0V for 5V Arduinos (Arduino Uno, etc.).
3V3 - This is the OUTPUT of the 3.3V regulator, and can be used to provide 3.3V power to other parts of your project if required (<100mA).
GND - Connect this to the GND pin on your development board to make sure they are sharing a common GND connection, 
      or the electrons won't have anywhere to flow!

SCL - The clock line on the I2C bus. This pin has an internal pullup resistor on the PCB, which is required as part of the I2C spec, 
      meaning you don't need to add one externally yourself. This also functions as SCK in SPI mode.
SDA - The data line on the I2C bus. This pin has an internal pullup resistor on the PCB, which is required as part of the I2C spec, 
      meaning you don't need to add one externally yourself. This also functions as MISO in SPI mode.
SDO/ALT ADDR - This pin can be used as MOSI in SPI mode, but is more commonly used as an optional bit in the I2C bus address.  
      By default this pin is pulled down, meaning it has a value of 0 at startup, which will results in an I2C address of 0x53. If you set this pin high (to 3.3V), and reset, the I2C address will be updated to 0x1D.
CS:  This dual purpose pin can be used as the chip select line in SPI mode, but also determines whether the board will boot up 
     into I2C or SPI mode. The default of logic high sets the board up for I2C, and manually setting this pin low and resetting will cause the device to enter SPI mode. Please note that SPI mode is not actively support and the SPI pins are not all 5V safe and level shifted, so care will be required when using it!
INT1 and INT2: There are two optional interrupt output pins on this sensor, which can be configured to change their state 
     when one or more 'events' occur. For details on how to use these interrupts, see the Arduino/HW Interrupts page later in this guide.

Activity/Inactivity Detection
Rather that constantly polling an accelerometer to see if movement is detected, you can configure the ADXL343 to let you know 
when there is (one or both of) activity or inactivity on the device, with user-adjustable thresholds. This can be configure 
to fire an INT pin, which you could use to wakeup your device, for example, or put it to sleep after a certain amount of inactivity.


*******************************************************************************/

#include "AnalogSmooth.h"
#include "Adafruit_ADT7410.h"
#include "Adafruit_ADXL343.h"
#include "../lib/FiniteStateMachine/src/FiniteStateMachine.h"

#define APP_NAME "beehiveMonitor"
#define VERSION "Version 0.06"

#define DEBUGGING

#define ALWAYS_ONLINE

// sleep if battery is low for some time (4 hours?) units: SECONDS
#define LOW_BATTERY_SLEEP 14400

// if not always online, sleep for this time - units: SECONDS
#define NORMAL_SLEEP_CYCLE 600

// #define USE_SOLAR_PANEL
#define USE_ADT7410
#define USE_ADXL343
#define USE_DS18B20
#define DS18B20_PIN D9

// comment out if you do NOT want serial logging
#ifdef DEBUGGING
SerialLogHandler logHandler(LOG_LEVEL_ALL);
#endif

/*******************************************************************************
 * changes in version 0.01:
       * Particle Build link: https://go.particle.io/shared_apps/5a6b8e6d7e1a22f899001161
       * Initial version
 * changes in version 0.02:
       * removed legacy Serial logs, added newer SerialLogHandler
       * removed use_adt7410, etc since they created trouble when building for boron or xenon
       * added adt7410 support
 * changes in version 0.03:
       * added adxl343 support
 * changes in version 0.04:
       * added interrupts with INPUT_PULLDOWN:
          pinMode(ADXL343_INPUT_PIN_INT1, INPUT_PULLDOWN);
 * changes in version 0.05:
       * added sleep and wake on pin interrupt
       * added sleep on low batt
 * changes in version 0.06:
       * adding location: #define LOCATION "bees1"

TODO:
 * check this xenon voltage formula out:
     battVoltage = analogRead(BATT) * 0.0011224 / 3.7 * 100;
     source: https://blog.particle.io/2019/06/26/get-started-with-ble-and-nfc/
*******************************************************************************/

#define LOCATION "bees1"

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

float batteryReading = 0;
bool useFahrenheit = true;

/*******************************************************************************
vvvv    DS18B20 related    vvvv
*******************************************************************************/
#ifdef USE_DS18B20
#include <DS18B20.h>
#include <math.h>
// Sets Pin DS18B20_PIN for DS18B20 Temp Sensor
// true: this is the only sensor on bus
DS18B20 ds18b20(DS18B20_PIN, true);

// let's increase the MAXRETRY as per this advise:
// https://community.particle.io/t/ds18b20-temperature-sensor-and-correct-library-for-boron/46933/5?u=gusgonnet
const int MAXRETRY = 30;

double temp_DS18B20_celsius;
double temp_DS18B20_fahrenheit;
#endif
/*******************************************************************************
^^^^    DS18B20 related    ^^^^
*******************************************************************************/

/*******************************************************************************
vvvv    ADT7410 related    vvvv
*******************************************************************************/
#ifdef USE_ADT7410
Adafruit_ADT7410 tempsensor_ADT7410 = Adafruit_ADT7410();

float temp_ADT7410_celsius;
float temp_ADT7410_fahrenheit;
#endif
/*******************************************************************************
^^^^    ADT7410 related    ^^^^
*******************************************************************************/

/*******************************************************************************
vvvv    ADXL343 related    vvvv
*******************************************************************************/
#ifdef USE_ADXL343
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

// in my understanding, this is the SENSITIVITY of the ACTIVITY DETECTION
// 0x20: if I tap on the table the board is, the device wakes up
// 0x50: I need to tap hard
#define ADXL343_SENSITIVITY 0x50

/** The input pin to enable the interrupt on, connected to INT1 on the ADXL. */
#define ADXL343_INPUT_PIN_INT1 A0

// leave only one
// #define ADXL343_RANGE ADXL343_RANGE_16_G
// #define ADXL343_RANGE ADXL343_RANGE_8_G
// #define ADXL343_RANGE ADXL343_RANGE_4_G
#define ADXL343_RANGE ADXL343_RANGE_2_G

/**
 * This struct is used to count the number of times that specific interrutps
 * have been fired by the ADXL and detected on the MCU. They will increment
 * by one for each event associated with the specified interrupt 'bit'.
 */
struct adxl_int_stats
{
  uint32_t data_ready;
  uint32_t single_tap;
  uint32_t double_tap;
  uint32_t activity;
  uint32_t inactivity;
  uint32_t freefall;
  uint32_t watermark;
  uint32_t overrun;
  uint32_t total;
};

/** Global stats block, incremented inside the interrupt handler(s). */
struct adxl_int_stats g_int_stats = {0};

/** Global counter to track the numbers of unused interrupts fired. */
uint32_t g_ints_fired = 0;

/** Global variable to determine which interrupt(s) are enabled on the ADXL343. */
int_config g_int_config_enabled = {0};

/** Global variables to determine which INT pin interrupt(s) are mapped to on the ADXL343. */
int_config g_int_config_map = {0};

#endif
/*******************************************************************************
^^^^    ADT7410 related    ^^^^
*******************************************************************************/

/*******************************************************************************
vvvv    Finite State Machine related    vvvv
*******************************************************************************/
#ifdef USE_ADXL343
// min amount of time to stay in alarm before coming back to normal in millis
// this delays the sending out of alarm
#define MOVEMENT_DETECTED_TIMEOUT 10000

#define STATE_OK "OK State"
#define STATE_ALARM "Alarm State"

// FSM declaration for water sensor
State accelerometerOkState = State(accelerometerOkEnterFunction, accelerometerOkUpdateFunction, accelerometerOkExitFunction);
State accelerometerAlarmState = State(accelerometerAlarmEnterFunction, accelerometerAlarmUpdateFunction, accelerometerAlarmExitFunction);
FSM accelerometerStateMachine = FSM(accelerometerOkState);
String accelerometerState = STATE_OK;
#endif

/*******************************************************************************
^^^^    Finite State Machine related    ^^^^
*******************************************************************************/

#define LITTLE 50

bool publishStatusFlag = false;
bool movementDetected = false;

// this variable is used to skip running code in always online devices
// when they come back from sleep we do not want to run code
// we want them to check the battery again just in case is still low
bool justCameBackFromSleep = false;

// This class allows to query the information about the latest System.sleep().
// if wake up reason is pin or timer
// https://docs.particle.io/reference/device-os/firmware/photon/#sleepresult-class
SleepResult result = System.sleepResult();

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{
  // cloud functions
  // Up to 15 cloud functions may be registered and each function name is limited to
  // a maximum of 12 characters (prior to 0.8.0), 64 characters (since 0.8.0).
  // https://docs.particle.io/reference/device-os/firmware/boron/#particle-function-
  Particle.function("forcePublishStatus", forcePublishStatus);

  // cloud variables
  // Up to 20 cloud variables may be registered and each variable name is limited to
  // a maximum of 12 characters (prior to 0.8.0), 64 characters (since 0.8.0).
  // https://docs.particle.io/reference/device-os/firmware/boron/#particle-variable-
#ifdef USE_ADXL343
  Particle.variable("AccelerometerState", accelerometerState);

  pinMode(ADXL343_INPUT_PIN_INT1, INPUT_PULLDOWN);
#endif

#ifdef USE_SOLAR_PANEL
#if PLATFORM_ID == PLATFORM_BORON
  PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below.

  // WARNING WARNING WARNING WARNING
  // WARNING WARNING WARNING WARNING
  // it seems this messes up the boron when no battery is connected
  pmic.setInputVoltageLimit(5080); //  for 6V Solar Panels
  // WARNING WARNING WARNING WARNING
  // WARNING WARNING WARNING WARNING

  pmic.setInputCurrentLimit(2000);         // 2000 mA, higher than req'd
  pmic.setChargeVoltage(4208);             //  Set Li-Po charge termination voltage to 4.21V,  Monitor the Enclosure Temps
  pmic.setChargeCurrent(0, 0, 1, 1, 1, 0); // 1408 mA [0+0+512mA+256mA+128mA+0] + 512 Offset
  pmic.enableDPDM();
#endif
#endif

#if PLATFORM_ID == PLATFORM_XENON
  pinMode(BATT, INPUT);
#endif

#ifdef USE_ADT7410
  tempsensor_ADT7410.begin();
#endif

#ifdef USE_ADXL343
  if (!accel.begin())
  {
    Log.error("Ooops, no ADXL343 detected ... Check your wiring!");
  }

  accelConfiguration();
#endif
}

/*******************************************************************************
 * Function Name  : loop for always online devices
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
#ifdef ALWAYS_ONLINE
void loop() // loop for always online devices
{

#if PLATFORM_ID == PLATFORM_XENON
  smoothBATT();
#endif

  // go to sleep if low battery
  checkLowBattery();

  // execute the code if not coming back from sleep
  // if the code is just coming back from sleep, and the sleep
  // was done due to low bat, we will potentially waste battery running this code
  if (not justCameBackFromSleep)
  {

#ifdef USE_ADXL343
    getAcceleration();
    // printAccelInfo();
    // accelDetected();
    accelerometerStateMachine.update();
#endif

    // check if the forcePublishStatus cloud function was called
    checkPublishStatusFlag();

#ifdef DEBUGGING
    delay(500);
#endif
  }
}
#endif

/*******************************************************************************
 * Function Name  : loop for devices that sleep
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
#ifndef ALWAYS_ONLINE
void loop() // loop for devices that sleep
{

#if PLATFORM_ID == PLATFORM_XENON
  smoothBATT();
#endif

  // go to sleep if low battery
  checkLowBattery();

#ifdef USE_ADXL343

#ifdef DEBUGGING
  // if I don't delay this, the Log.info below does not print anything since the serial port
  // takes a bit to connect to the device
  delay(8000);
#endif

  if (result.wokenUpByPin())
  {
    Log.info("Device was woken up by a pin");
    movementDetected = true;
  }

  if (result.wokenUpByRtc())
  {
    Log.info("Device was woken up by the timer");
    movementDetected = false;
  }

#endif

  // publish the info before going to sleep
  publishStatus();

  System.sleep(ADXL343_INPUT_PIN_INT1, RISING, NORMAL_SLEEP_CYCLE);
  result = System.sleepResult();
}
#endif

/*******************************************************************************
 * Function Name  : publishStatus
 * Description    : publish status message
 *******************************************************************************/
void publishStatus()
{
#ifdef USE_DS18B20
  getTempDS18B20();
#endif

#ifdef USE_ADT7410
  getTempADT7410();
#endif

#define BUFFER 623
  char pubChar[BUFFER] = "";
  snprintf(pubChar, BUFFER, "Location: %s, Movement: %d", LOCATION, movementDetected);

  char tempChar[LITTLE] = "";
#if PLATFORM_ID == PLATFORM_BORON
  snprintf(tempChar, LITTLE, ", SoC: %.2f%%", batteryMonitor.getSoC());
  strcat(pubChar, tempChar);
#endif

#if PLATFORM_ID == PLATFORM_XENON
  snprintf(tempChar, LITTLE, ", Battery: %.2f Volts", batteryReading);
  strcat(pubChar, tempChar);
#endif

#ifdef USE_ADT7410
  if (useFahrenheit)
  {
    snprintf(tempChar, LITTLE, ", adt7410: %.2f", temp_ADT7410_fahrenheit);
  }
  else
  {
    snprintf(tempChar, LITTLE, ", adt7410: %.2f", temp_ADT7410_celsius);
  }
  strcat(pubChar, tempChar);
#endif

#ifdef USE_DS18B20
  if (useFahrenheit)
  {
    snprintf(tempChar, LITTLE, ", ds18b20: %.2f", temp_DS18B20_fahrenheit);
  }
  else
  {
    snprintf(tempChar, LITTLE, ", ds18b20: %.2f", temp_DS18B20_celsius);
  }
  strcat(pubChar, tempChar);
#endif

  Log.info(pubChar);
  Particle.publish("STATUS", pubChar, PRIVATE | WITH_ACK);

#ifndef ALWAYS_ONLINE
  // leave time to publish to go out
  delay(4000);
#endif
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
 * Function Name  : checkLowBattery
 * Description    : protect the device from low batt situations
 *******************************************************************************/
void checkLowBattery()
{
  justCameBackFromSleep = false;

#if PLATFORM_ID == PLATFORM_BORON
  // send it to sleep if no more battery
  if (batteryMonitor.getSoC() < CRITICAL_BATTERY_BORON)
  {
    System.sleep(ADXL343_INPUT_PIN_INT1, RISING, LOW_BATTERY_SLEEP);
    justCameBackFromSleep = true;
  }
#endif
#if PLATFORM_ID == PLATFORM_XENON
  // send it to sleep if no more battery
  if (batteryReading < CRITICAL_BATTERY_XENON)
  {
    System.sleep(ADXL343_INPUT_PIN_INT1, RISING, LOW_BATTERY_SLEEP);
    justCameBackFromSleep = true;
  }
#endif
}

/*******************************************************************************
 * Function Name  : getTempDS18B20
 * Description    : get reading from the DS18b20 sensor
 *******************************************************************************/
#ifdef USE_DS18B20
void getTempDS18B20()
{
  float _temp;
  int i = 0;
  char tempChar[LITTLE] = "";

  do
  {
    _temp = ds18b20.getTemperature();
    // Log.info(i);
    // Log.info(_temp);
    // Log.info(ds18b20.crcCheck());

  } while (((_temp == NAN) || !ds18b20.crcCheck()) && MAXRETRY > i++);

  if (i < MAXRETRY)
  {
    temp_DS18B20_celsius = _temp;
    temp_DS18B20_fahrenheit = ds18b20.convertToFahrenheit(_temp);
  }
  else
  {
    temp_DS18B20_celsius = temp_DS18B20_fahrenheit = NAN;
    Log.info("Invalid reading");
  }

  snprintf(tempChar, LITTLE, "ds18b20 fahrenheit: %.2f", temp_DS18B20_fahrenheit);
  Log.info(tempChar);

  snprintf(tempChar, LITTLE, "ds18b20 celsius: %.2f", temp_DS18B20_celsius);
  Log.info(tempChar);
}
#endif

/*******************************************************************************
 * Function Name  : getTempADT7410
 * Description    : get reading from the ADT7410 sensor
 *******************************************************************************/
#ifdef USE_ADT7410
void getTempADT7410()
{

  temp_ADT7410_celsius = tempsensor_ADT7410.readTempC();
  temp_ADT7410_fahrenheit = (temp_ADT7410_celsius * 1.8) + 32.0;

  char tempChar[LITTLE] = "";
  snprintf(tempChar, LITTLE, "adt7410 fahrenheit: %.2f", temp_ADT7410_fahrenheit);
  Log.info(tempChar);

  snprintf(tempChar, LITTLE, "adt7410 celsius: %.2f", temp_ADT7410_celsius);
  Log.info(tempChar);
}
#endif

/*******************************************************************************
 * Function Name  : displaySensorDetails
 * Description    : display some info from the accelerometer
 *******************************************************************************/
#ifdef USE_ADXL343
void displaySensorDetails()
{
  char tempChar[LITTLE] = "";

  sensor_t sensor;
  accel.getSensor(&sensor);
  Log.info("------------------------------------");
  Log.info("Sensor:       ");
  Log.info(sensor.name);
  snprintf(tempChar, LITTLE, "Driver Ver: %li", sensor.version);
  Log.info(tempChar);
  // Log.info("Unique ID:    ");
  // Log.info(sensor.sensor_id);
  // Log.info("Max Value:    ");
  // Log.info(sensor.max_value); Log.info(" m/s^2");
  // Log.info("Min Value:    ");
  // Log.info(sensor.min_value); Log.info(" m/s^2");
  // Log.info("Resolution:   ");
  // Log.info(sensor.resolution); Log.info(" m/s^2");
  Log.info("------------------------------------");
}

/*******************************************************************************
 * Function Name  : getAcceleration
 * Description    : get more info from the accelerometer
 *******************************************************************************/
void getAcceleration()
{
  sensors_event_t event;
  accel.getEvent(&event);
  char tempChar[LITTLE] = "";

  /* Display the results (acceleration is measured in m/s^2) */
  Log.info("------------------------------------");
  snprintf(tempChar, LITTLE, "X: %.2f", event.acceleration.x);
  Log.info(tempChar);
  snprintf(tempChar, LITTLE, "y: %.2f", event.acceleration.y);
  Log.info(tempChar);
  snprintf(tempChar, LITTLE, "z: %.2f", event.acceleration.z);
  Log.info(tempChar);
  Log.info("------------------------------------");
  Log.info(VERSION);

  delay(500);
}

/** Interrupt service routine for INT1 events. */
// source: https://learn.adafruit.com/adxl343-breakout-learning-guide/hw-interrupts
void adxl343_int1_isr(void)
{
  // Log.info("isr triggered");
  g_int_stats.activity++;
  g_int_stats.total++;
  g_ints_fired++;
  movementDetected = true;
}

void printAccelInfo()
{
  char tempChar[LITTLE] = "";

  uint8_t format = accel.readRegister(ADXL343_REG_INT_ENABLE);
  snprintf(tempChar, LITTLE, "read ADXL343_REG_INT_ENABLE: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_INT_MAP);
  snprintf(tempChar, LITTLE, "read ADXL343_REG_INT_MAP: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_INT_SOURCE);
  snprintf(tempChar, LITTLE, "read ADXL343_REG_INT_SOURCE: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_THRESH_ACT);
  snprintf(tempChar, LITTLE, "read ADXL343_REG_THRESH_ACT: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_ACT_INACT_CTL);
  snprintf(tempChar, LITTLE, "read ADXL343_REG_ACT_INACT_CTL: %i", format);
  Log.info(tempChar);
}

/** Configures range, HW interrupts on the ADXL343 and the target MCU. */
// source: https://learn.adafruit.com/adxl343-breakout-learning-guide/hw-interrupts
void accelConfiguration(void)
{
  /* NOTE: Once an interrupt fires on the ADXL you can read a register
   *  to know the source of the interrupt, but since this would likely
   *  happen in the 'interrupt context' performing an I2C read is a bad
   *  idea since it will block the device from handling other interrupts
   *  in a timely manner.
   *
   *  The best approach is to try to make use of only two interrupts on
   *  two different interrupt pins, so that when an interrupt fires, based
   *  on the 'isr' function that is called, you already know the int source.
   */

  // this is not needed when the device sleeps
#ifdef ALWAYS_ONLINE
  /* Attach interrupt inputs on the MCU. */
  if (not attachInterrupt(ADXL343_INPUT_PIN_INT1, adxl343_int1_isr, RISING))
  // if (not attachInterrupt(ADXL343_INPUT_PIN_INT1, adxl343_int1_isr, CHANGE))
  {
    Log.error("Could not attach interrupt to pin!");
  }
#endif

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE);

  /* Enable interrupts on the accelerometer. */
  g_int_config_enabled.bits.overrun = false;
  g_int_config_enabled.bits.watermark = false;
  g_int_config_enabled.bits.freefall = false;
  g_int_config_enabled.bits.inactivity = false;
  g_int_config_enabled.bits.activity = true; /* Set the INT1 */
  g_int_config_enabled.bits.double_tap = false;
  g_int_config_enabled.bits.single_tap = false;
  g_int_config_enabled.bits.data_ready = false;
  accel.enableInterrupts(g_int_config_enabled);

  /* Map specific interrupts to one of the two INT pins. */
  g_int_config_map.bits.overrun = ADXL343_INT1;
  g_int_config_map.bits.watermark = ADXL343_INT1;
  g_int_config_map.bits.freefall = ADXL343_INT1;
  g_int_config_map.bits.inactivity = ADXL343_INT1;
  g_int_config_map.bits.activity = ADXL343_INT1;
  g_int_config_map.bits.double_tap = ADXL343_INT1;
  g_int_config_map.bits.single_tap = ADXL343_INT1;
  g_int_config_map.bits.data_ready = ADXL343_INT2;
  accel.mapInterrupts(g_int_config_map);

  // Register 0x27—ACT_INACT_CTL (Read/Write)
  // D7        | D6           | D5           | D4
  // ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable
  //
  // D3          | D2             | D1             | D0
  // INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable
  //
  // ACT AC/DC and INACT AC/DC Bits
  // A setting of 0 selects dc-coupled operation, and a setting of 1
  // enables ac-coupled operation.
  //
  // In dc-coupled operation, the
  // current acceleration magnitude is compared directly with
  // THRESH_ACT and THRESH_INACT to determine whether
  // activity or inactivity is detected.
  //
  // In ac-coupled operation for activity detection, the acceleration
  // value at the start of activity detection is taken as a reference
  // value. New samples of acceleration are then compared to this
  // reference value, and if the magnitude of the difference exceeds
  // the THRESH_ACT value, the device triggers an activity interrupt.
  //
  // in my understanding, the Z axis will read a value of ~9, which
  // indicates the acceleration of the gravity, hence if the board is installed
  // with the accelerometer FLAT, we need to deactivate the z axis trigger
  // of activity, hence here I write 0110 000 (0x60) which enables only axis x and y
  accel.writeRegister(ADXL343_REG_ACT_INACT_CTL, 0x60);

  // Register 0x24—THRESH_ACT (Read/Write)
  // The THRESH_ACT register is eight bits and holds the threshold
  // value for detecting activity. The data format is unsigned,
  // therefore, the magnitude of the activity event is compared
  // with the value in the THRESH_ACT register. The scale factor
  // is 62.5 mg/LSB. A value of 0 may result in undesirable behavior
  // if the activity interrupt is enabled.
  //
  // in my understanding, this is the SENSITIVITY of the ACTIVITY DETECTION
  // 0x20: if I tap on the table the board is, the device wakes up
  accel.writeRegister(ADXL343_REG_THRESH_ACT, ADXL343_SENSITIVITY);
}

void accelDetected()
{
  while (g_ints_fired)
  {
    Log.info("ACTIVITY detected!");
    /* Decrement the unhandled int counter. */
    g_ints_fired--;
  }
}

#endif

/*******************************************************************************
********************************************************************************
********************************************************************************
 FINITE STATE MACHINE FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

void accelerometerOkEnterFunction()
{
#ifdef DEBUGGING
  Particle.publish("OK", "Accelerometer is OK", PRIVATE | WITH_ACK);
#endif
  accelerometerSetState(STATE_OK);
}
void accelerometerOkUpdateFunction()
{

  while (g_ints_fired)
  {
    Log.info("ACTIVITY detected!");
    /* Decrement the unhandled int counter. */
    g_ints_fired--;
  }

  if (movementDetected)
  {
    accelerometerStateMachine.transitionTo(accelerometerAlarmState);
    Log.info("Movement detected, transition to accelerometerAlarmState");
  }
}
void accelerometerOkExitFunction()
{
}

void accelerometerAlarmEnterFunction()
{
#ifdef DEBUGGING
  Particle.publish("ALARM", "Accelerometer detected movement", PRIVATE | WITH_ACK);
#endif
  accelerometerSetState(STATE_ALARM);
}
void accelerometerAlarmUpdateFunction()
{
  // stay here a minimum time
  if (accelerometerStateMachine.timeInCurrentState() < MOVEMENT_DETECTED_TIMEOUT)
  {
    return;
  }

  // publish the info before going to ok state
  publishStatus();

  accelerometerStateMachine.transitionTo(accelerometerOkState);
  Log.info("Alarm sent, transition to accelerometerOkState");
}
void accelerometerAlarmExitFunction()
{
  // clear flag of fired accel events
  g_ints_fired = 0;
  movementDetected = false;
}

/*******************************************************************************
 * Function Name  : accelerometerSetState
 * Description    : sets the state of an FSM
 * Return         : none
 *******************************************************************************/
void accelerometerSetState(String newState)
{
  accelerometerState = newState;
#ifdef DEBUGGING
  Particle.publish("FSM", "Accelerometer fsm entering " + newState + " state", PRIVATE | WITH_ACK);
#endif
  Log.info("Accelerometer fsm entering " + newState + " state");
}
