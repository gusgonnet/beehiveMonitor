/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/home/ceajog/0trabajo/omgbees/beehiveMonitor/src/beehiveMonitor.ino"
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
#include "../lib/PublishQueueAsyncRK/src/PublishQueueAsyncRK.h"
#include "../lib/JsonParserGeneratorRK/src/JsonParserGeneratorRK.h"

/*******************************************************************************
********************************************************************************
********************************************************************************
USER CAN CHANGE THESE DEFINES BELOW
********************************************************************************
********************************************************************************
*******************************************************************************/

// this is used to identify the device and beehive when it publishes to the cloud
String firmwareVersion();
void setup();
void loop();
void loop();
void sendDataToUbidots(bool scheduled);
void readSensors(bool scheduled);
void publishStatus();
void checkPublishStatusFlag();
int forcePublishStatus(String dummy);
void checkLowBattery();
void getTempDS18B20();
void getTempADT7410();
void displaySensorDetails();
void getAcceleration();
void adxl343_int1_isr(void);
void printAccelInfo();
void accelConfiguration(void);
void accelDetected();
void accelerometerInitEnterFunction();
void accelerometerInitUpdateFunction();
void accelerometerInitExitFunction();
void accelerometerOkEnterFunction();
void accelerometerOkUpdateFunction();
void accelerometerOkExitFunction();
void accelerometerAlarmEnterFunction();
void accelerometerAlarmUpdateFunction();
void accelerometerAlarmExitFunction();
void accelerometerSetState(String newState);
#line 70 "/home/ceajog/0trabajo/omgbees/beehiveMonitor/src/beehiveMonitor.ino"
#define BEEHIVE_LOCATION "bees1"

// this determines if the device will always be on.
// if commented out, the device will sleep and wake either:
//  - on movement detected at any time
//  - every NORMAL_SLEEP_CYCLE (4hs) to report periodically to the cloud
#define ALWAYS_ONLINE

// if not always online, the device will sleep for this time. It reports status to the cloud every time it wakes up.
// units: MINUTES (example: 240 minutes => 4 hours)
#define NORMAL_SLEEP_CYCLE 240

// publish to ubidots every number of minutes
#define PUBLISH_TO_UBIDOTS 240

// webhook to send data to ubidots
// you can create this webhook as explained here:
// https://help.ubidots.com/en/articles/513304-connect-your-particle-device-to-ubidots-using-particle-webhooks
#define WEBHOOK_NAME "ubidotsbees"

// sleep if battery is low for some time
// units: MINUTES (example: 240 minutes => 4 hours)
#define LOW_BATTERY_SLEEP 240

// threshold in percentage
#define CRITICAL_BATTERY 20

// how often to read the sensors
#define READ_SENSORS_SECONDS 5

// here you can configure what sensors you have connected
#define USE_ADT7410 // temp sensor
#define USE_ADXL343 // accel sensor
#define USE_DS18B20 // temp sensor

// this defines the pin you connected the DS18B20 temperature sensor
#define DS18B20_PIN D9

// this defines if the temperature is preferred in fahrenheit
// comment out for celsius
#define TEMP_IN_FAHRENHEIT true

// how long to wait for cloud connection when the device wakes up
// used in devices that sleep and wake every 4 hours (ALWAYS_ONLINE not defined)
#define WAIT_FOR_PARTICLE_CONNECT 15

// define only if you are debugging your code
// WARNING
// WARNING: it may change behaviour (timeouts, sleep periods)
//          so do not deploy in the field a device in debug
// WARNING
#define DEBUGGING

/*******************************************************************************
********************************************************************************
********************************************************************************
END -> USER CAN CHANGE THESE DEFINES ABOVE
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * changes in version 0.01:
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
       * adding location: #define BEEHIVE_LOCATION "bees1"
 * changes in version 0.07:
       * removing Xenon, updating code to Device OS 2.0.1, updating libs to latest version
 * changes in version 0.08:
       * adding cloud variable with firmware version
       * Removed reference to PMIC settings
       * adding new sleep class in device os 2.x
 * changes in version 0.09:
       * adding ubidots
          source: https://help.ubidots.com/en/articles/513304-connect-your-particle-device-to-ubidots-using-particle-webhooks
 * changes in version 0.10:
       * adding sendDataToUbidots()
       * renamed cloud function forcePublishStatus to updateUbidotsRightAway
       * adding PublishQueueAsyncRK to send alarms in a more reliable way
 * changes in version 0.11:
       * removing ubidots lib in favor of using PublishQueueAsyncRK to trigger the webhook to ubidots
       * adding JsonParserGeneratorRK
       * removing ubidots library
       * adding WAIT_FOR_PARTICLE_CONNECT


How to create the Particle webhook to Ubidots:
https://help.ubidots.com/en/articles/513304-connect-your-particle-device-to-ubidots-using-particle-webhooks

*******************************************************************************/
String firmwareVersion()
{
  return "BeehiveMonitor - Version 0.11";
}

//enable the user code (our program below) to run in parallel with cloud connectivity code
// source: https://docs.particle.io/reference/firmware/photon/#system-thread
SYSTEM_THREAD(ENABLED);

retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));

SerialLogHandler logHandler(LOG_LEVEL_INFO);

// if the user commented out the fahrenheit define above, define a celsius one
#ifndef TEMP_IN_FAHRENHEIT
#define TEMP_IN_FAHRENHEIT false
#endif

#define MILLISECONDS_TO_SECONDS 1000
#define MILLISECONDS_TO_MINUTES 60000
#define SMALL_BUFFER 50

#if PLATFORM_ID == PLATFORM_BORON
FuelGauge batteryMonitor;
#endif

bool publishStatusFlag = false;
bool movementDetected = false;

// this variable is used to skip running code in always online devices
// when they come back from sleep we do not want to run code in loop()
// we want them to check the battery again just in case is still low
bool sleepingDueToLowBatt = false;

// This class allows to query the information about the latest System.sleep().
// if wake up reason is pin or timer
// https://docs.particle.io/reference/device-os/firmware/argon/#systemsleepresult-class
SystemSleepResult result;

#define RIGHT_NOW false
#define SCHEDULED true

// this initial value ensures the device updates ubidots right after a reset
unsigned long ubidotsTime = PUBLISH_TO_UBIDOTS * MILLISECONDS_TO_MINUTES * 2;

unsigned long debugTime = 0;

unsigned long readSensorsTime = 0;

// redefine the sleep and low battery cycles, the publish to ubidots period
#ifdef DEBUGGING
#undef NORMAL_SLEEP_CYCLE
#define NORMAL_SLEEP_CYCLE 5

#undef LOW_BATTERY_SLEEP
#define LOW_BATTERY_SLEEP 5

#undef PUBLISH_TO_UBIDOTS
#define PUBLISH_TO_UBIDOTS 5

#endif

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
 * This struct is used to count the number of times that specific interrupts
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

#define INIT_TIMEOUT 10000

#define STATE_INIT "Init State"
#define STATE_OK "OK State"
#define STATE_ALARM "Alarm State"

// FSM declaration for water sensor
State accelerometerInitState = State(accelerometerInitEnterFunction, accelerometerInitUpdateFunction, accelerometerInitExitFunction);
State accelerometerOkState = State(accelerometerOkEnterFunction, accelerometerOkUpdateFunction, accelerometerOkExitFunction);
State accelerometerAlarmState = State(accelerometerAlarmEnterFunction, accelerometerAlarmUpdateFunction, accelerometerAlarmExitFunction);
FSM accelerometerStateMachine = FSM(accelerometerInitState);
String accelerometerState = STATE_INIT;
#endif

/*******************************************************************************
^^^^    Finite State Machine related    ^^^^
*******************************************************************************/

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
  Particle.function("updateUbidotsRightAway", forcePublishStatus);

  // cloud variables (and calculated)
  // Up to 20 cloud variables may be registered and
  // each variable name is limited to a maximum of 12 characters (prior to 0.8.0), 64 characters (since 0.8.0).
  // It is also possible to register a function to compute a cloud variable.
  // https://docs.particle.io/reference/device-os/firmware/boron/#particle-variable-
  Particle.variable("firmwareVersion", firmwareVersion);

  // cloud variables
  // Up to 20 cloud variables may be registered and each variable name is limited to
  // a maximum of 12 characters (prior to 0.8.0), 64 characters (since 0.8.0).
  // https://docs.particle.io/reference/device-os/firmware/boron/#particle-variable-
#ifdef USE_ADXL343
  Particle.variable("AccelerometerState", accelerometerState);

  pinMode(ADXL343_INPUT_PIN_INT1, INPUT_PULLDOWN);
#endif

#ifdef USE_ADT7410
  // tempsensor_ADT7410.begin();
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor_ADT7410.begin())
  {
    Log.error("Ooops, no ADT7410 temperature sensor detected. Please check your wiring!");
    // TODO: not sure if this while loop works fine - TO BE TESTED!
    while (1)
      ;
  }
#endif

#ifdef USE_ADXL343
  if (!accel.begin())
  {
    Log.error("Ooops, no ADXL343 accelerometer detected. Please check your wiring!");
    // TODO: not sure if this while loop works fine - TO BE TESTED!
    while (1)
      ;
  }

  accelConfiguration();
#endif

  // visual debug
  pinMode(D7, OUTPUT);
}

/*******************************************************************************
 * Function Name  : loop for always online devices
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
#ifdef ALWAYS_ONLINE
void loop() // loop for always online devices
{

#if PLATFORM_ID == PLATFORM_BORON

  // go to sleep if low battery
  checkLowBattery();

  // if the device is just waking up from a low bat situation
  //     => we return since this will call again loop() and check the battery again
  // this will avoid wasting battery by running the rest of the loop() function
  if (sleepingDueToLowBatt)
  {
    return;
  }
#endif

  readSensors(SCHEDULED);

#ifdef USE_ADXL343
  accelerometerStateMachine.update();
#endif

  // check if the forcePublishStatus cloud function was called
  checkPublishStatusFlag();

  // this publishes status and propagates alarms and data to ubidots
  sendDataToUbidots(SCHEDULED);

#ifdef DEBUGGING
  // send debug publish every 10 seconds
  if (millis() - debugTime > 10000)
  {
    publishStatus();
    debugTime = millis();
  }
#endif
}
#endif

/*******************************************************************************
 * Function Name  : loop for devices that sleep
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
#ifndef ALWAYS_ONLINE
void loop() // loop for devices that sleep
{

#if PLATFORM_ID == PLATFORM_BORON

  // go to sleep if low battery
  checkLowBattery();

  // if the device is just waking up from a low bat situation
  //     => we return since this will call again loop() and check the battery again
  // this will avoid wasting battery by running the rest of the loop() function
  if (sleepingDueToLowBatt)
  {
    return;
  }

#endif

  // wait up to 15 minutes for the device to connect to the particle cloud
  if (waitFor(Particle.connected, WAIT_FOR_PARTICLE_CONNECT * MILLISECONDS_TO_MINUTES))
  {
    Log.info("Device connected to the Particle cloud");
  }
  else
  {
    Log.info("Error: device could NOT connect to the Particle cloud");
  }

  // #ifdef DEBUGGING
  //   // if I don't delay this, the Log.info below does not print anything since the serial port
  //   // takes a bit to connect to the device
  //   delay(20000);
  // #endif

  readSensors(RIGHT_NOW);

#ifdef USE_ADXL343

  if (result.wakeupReason() == SystemSleepWakeupReason::BY_GPIO)
  {
    Log.info("Device was woken up by a pin => movement detected!");
    movementDetected = true;
  }

  if (result.wakeupReason() == SystemSleepWakeupReason::BY_RTC)
  {
    Log.info("Device was woken up by the timer => periodic report status to the cloud");
    movementDetected = false;
  }

#endif

  // publish the info before going to sleep
  sendDataToUbidots(RIGHT_NOW);

  // #ifdef DEBUGGING
  //   // if I don't delay this, the Log.info below does not print anything since the serial port
  //   // takes a bit to connect to the device
  //   delay(20000);
  // #endif

  SystemSleepConfiguration config;
  config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .duration(NORMAL_SLEEP_CYCLE * MILLISECONDS_TO_MINUTES)
      .gpio(ADXL343_INPUT_PIN_INT1, RISING)
      .flag(SystemSleepFlag::WAIT_CLOUD);
  result = System.sleep(config);
}
#endif

/*******************************************************************************
 * Function Name  : sendDataToUbidots
 * Description    : send the data to ubidots via webhook

 NOTE: this is what gets sent:
 {"temp_ds18b20":{"value":68.9},"temp_adt7410":{"value":69.9125},"movement":{"value":1}}
 *******************************************************************************/
void sendDataToUbidots(bool scheduled)
{

#ifdef ALWAYS_ONLINE
  // if the FSM is in init state, the device just rebooted so we do not send anything
  // since it can be invalid info
  // only valid in the case of always on devices
  // sleepy devices WAKE UP on an interrupt from the accelerometer, so we got the info already
  if (accelerometerStateMachine.isInState(accelerometerInitState))
  {
    return;
  }
#endif

  // if scheduled is true, we send to ubidots acording to schedule
  // (default 240 minutes == 4 hours)
  // if false, we send to ubidots right away => in case of alarm, for instance
  if (scheduled)
  {
    // source: https://github.com/kennethlimcp/particle-examples/blob/master/vbatt-argon-boron/vbatt-argon-boron.ino
    if (millis() - ubidotsTime < (PUBLISH_TO_UBIDOTS * MILLISECONDS_TO_MINUTES))
    {
      return;
    }
  }

// this blinks the onboard LED to flag we are about to send data to ubidots
#ifdef DEBUGGING
  digitalWrite(D7, HIGH);
  delay(250);
  digitalWrite(D7, LOW);
#endif

  ubidotsTime = millis();

  // This creates a buffer to hold up to 256 bytes of JSON data (good for Particle.publish)
  JsonWriterStatic<256> jw;

  // set decimals on json generator for double and float types
  jw.setFloatPlaces(1);

  // Creating a scope like this in {} with a JsonWriterAutoObject in it creates an object,
  // and automatically closes the object when leaving the scope. This is necessary because
  // all JSON values must be in either an object or an array to be valid, and JsonWriter
  // requires all startObject to be balanced with a finishObjectOrArray and JsonWriterAutoObject
  // takes care of doing that automatically.
  {
    JsonWriterAutoObject obj(&jw);

    // add temperatures from both DS18B20 and adt7410 sensors
    if (TEMP_IN_FAHRENHEIT)
    {
      // if the ds18b20 is not connected we get NAN
      // if we try to send that value, ubidots complains and rejects the message
      if (!isnan(temp_DS18B20_fahrenheit))
      {
        jw.insertKeyValue("temp_ds18b20", temp_DS18B20_fahrenheit);
      }

      jw.insertKeyValue("temp_adt7410", temp_ADT7410_fahrenheit);
    }
    else
    {

      // if the ds18b20 is not connected we get NAN
      // if we try to send that value, ubidots complains and rejects the message
      if (!isnan(temp_DS18B20_celsius))
      {
        jw.insertKeyValue("temp_ds18b20", temp_DS18B20_celsius);
      }

      jw.insertKeyValue("temp_adt7410", temp_ADT7410_celsius);
    }

    // add accel info
    jw.insertKeyValue("movement", movementDetected ? 1 : 0);

    // add charge
#if PLATFORM_ID == PLATFORM_BORON
    float batterySoc = 100;
    batterySoc = System.batteryCharge();
    jw.insertKeyValue("SoC", batterySoc);

    jw.insertKeyValue("State", System.batteryState());

#endif
  }

  bool messageEnqueued = publishQueue.publish(WEBHOOK_NAME, jw.getBuffer(), 60, PRIVATE, WITH_ACK);

  if (messageEnqueued)
  {
    Log.info("Ubidots webhook enqueued");
  }
  else
  {
    Log.info("Error: problems while enqueuing webhook to Ubidots");
  }

  publishStatus();
}

/*******************************************************************************
 * Function Name  : readSensors
 * Description    : read temperature sensors
 *******************************************************************************/
void readSensors(bool scheduled)
{

  // if scheduled is true, we read sensors acording to schedule (default every 5 seconds)
  // if false, we read sensors right away => in case of a device that sleeps all the time, for instance
  if (scheduled)
  {

    if (millis() - readSensorsTime < (READ_SENSORS_SECONDS * MILLISECONDS_TO_SECONDS))
    {
      return;
    }
  }

  readSensorsTime = millis();

  Log.info("About to read sensors");

#ifdef USE_DS18B20
  getTempDS18B20();
#endif

#ifdef USE_ADT7410
  getTempADT7410();
#endif

#ifdef USE_ADXL343
  getAcceleration();
  // printAccelInfo();
  // accelDetected();
#endif
}

/*******************************************************************************
 * Function Name  : publishStatus
 * Description    : publish status message to serial and particle cloud
 *******************************************************************************/
void publishStatus()
{
  Log.info("------------------------------------------");

#define BUFFER 623

  char pubChar[BUFFER] = "";
  snprintf(pubChar, BUFFER, "Location: %s, Movement: %d", BEEHIVE_LOCATION, movementDetected);

  char tempChar[SMALL_BUFFER] = "";

#if PLATFORM_ID == PLATFORM_BORON
  float batterySoc = System.batteryCharge();
  snprintf(tempChar, SMALL_BUFFER, ", SoC: %.2f%%", batterySoc);
  strcat(pubChar, tempChar);

  const char *batteryContext[7] = {"Unknown", "Not Charging", "Charging", "Charged", "Discharging", "Fault", "Diconnected"};
  // Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
  snprintf(tempChar, SMALL_BUFFER, ", State: %s", batteryContext[System.batteryState()]);
  strcat(pubChar, tempChar);

#endif

#ifdef USE_ADT7410
  if (TEMP_IN_FAHRENHEIT)
  {
    snprintf(tempChar, SMALL_BUFFER, ", adt7410: %.2f", temp_ADT7410_fahrenheit);
  }
  else
  {
    snprintf(tempChar, SMALL_BUFFER, ", adt7410: %.2f", temp_ADT7410_celsius);
  }
  strcat(pubChar, tempChar);
#endif

#ifdef USE_DS18B20
  if (TEMP_IN_FAHRENHEIT)
  {
    snprintf(tempChar, SMALL_BUFFER, ", ds18b20: %.2f", temp_DS18B20_fahrenheit);
  }
  else
  {
    snprintf(tempChar, SMALL_BUFFER, ", ds18b20: %.2f", temp_DS18B20_celsius);
  }
  strcat(pubChar, tempChar);
#endif

  Log.info(pubChar);
  Particle.publish("STATUS", pubChar, PRIVATE | WITH_ACK);
}

/*******************************************************************************
 * Function Name  : checkPublishStatusFlag
 * Description    : call publish status message if the publishStatus() cloud function was called
 *                  it sends info to the particle cloud and to Ubidots on demand
 *                  on demand: means the user called the forcePublishStatus() function
 *******************************************************************************/
void checkPublishStatusFlag()
{
  if (publishStatusFlag)
  {
    publishStatusFlag = false;
    publishStatus();
    sendDataToUbidots(RIGHT_NOW);
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
 * Function Name  : checkLowBattery
 * Description    : protect the device from low batt situations
 *******************************************************************************/
void checkLowBattery()
{
  sleepingDueToLowBatt = false;

  float batterySoc = 100;

#if PLATFORM_ID == PLATFORM_BORON
  batterySoc = System.batteryCharge();
#endif

  // something is wrong, try again in few cycles
  if (batterySoc < 0)
  {
    Log.info("SoC reported -1");
    return;
  }

  // send it to sleep if no more battery
  float batteryCritical = CRITICAL_BATTERY;
  if (batterySoc < batteryCritical)
  {

    Log.info("Battery too low, going to sleep");

// visual indication that the device will go to sleep
#ifdef DEBUGGING
    digitalWrite(D7, HIGH);
    delay(250);
    digitalWrite(D7, LOW);
    delay(250);
    digitalWrite(D7, HIGH);
    delay(250);
    digitalWrite(D7, LOW);
    delay(250);
    digitalWrite(D7, HIGH);
    delay(250);
    digitalWrite(D7, LOW);
#endif

    SystemSleepConfiguration config;

    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
        .duration(LOW_BATTERY_SLEEP * MILLISECONDS_TO_MINUTES)
        .gpio(ADXL343_INPUT_PIN_INT1, RISING)
        .flag(SystemSleepFlag::WAIT_CLOUD);
    result = System.sleep(config);

    sleepingDueToLowBatt = true;
  }
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
  char tempChar[SMALL_BUFFER] = "";

  do
  {
    _temp = ds18b20.getTemperature();

  } while (((_temp == NAN) || !ds18b20.crcCheck()) && MAXRETRY > i++);

  if (i < MAXRETRY)
  {
    temp_DS18B20_celsius = _temp;
    temp_DS18B20_fahrenheit = ds18b20.convertToFahrenheit(_temp);
  }
  else
  {
    temp_DS18B20_celsius = temp_DS18B20_fahrenheit = NAN;
    Log.info("ds18b20 error: invalid reading");
  }

  snprintf(tempChar, SMALL_BUFFER, "ds18b20 fahrenheit: %.2f", temp_DS18B20_fahrenheit);
  Log.info(tempChar);

  snprintf(tempChar, SMALL_BUFFER, "ds18b20 celsius: %.2f", temp_DS18B20_celsius);
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

  char tempChar[SMALL_BUFFER] = "";
  snprintf(tempChar, SMALL_BUFFER, "adt7410 fahrenheit: %.2f", temp_ADT7410_fahrenheit);
  Log.info(tempChar);

  snprintf(tempChar, SMALL_BUFFER, "adt7410 celsius: %.2f", temp_ADT7410_celsius);
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
  char tempChar[SMALL_BUFFER] = "";

  sensor_t sensor;
  accel.getSensor(&sensor);
  Log.info("------------------------------------");
  Log.info("Sensor:       ");
  Log.info(sensor.name);
  snprintf(tempChar, SMALL_BUFFER, "Driver Ver: %li", sensor.version);
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
  char tempChar[SMALL_BUFFER] = "";

  /* Display the results (acceleration is measured in m/s^2) */
  Log.info("------------------------------------");
  snprintf(tempChar, SMALL_BUFFER, "X: %.2f", event.acceleration.x);
  Log.info(tempChar);
  snprintf(tempChar, SMALL_BUFFER, "y: %.2f", event.acceleration.y);
  Log.info(tempChar);
  snprintf(tempChar, SMALL_BUFFER, "z: %.2f", event.acceleration.z);
  Log.info(tempChar);
  Log.info("------------------------------------");

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
  char tempChar[SMALL_BUFFER] = "";

  uint8_t format = accel.readRegister(ADXL343_REG_INT_ENABLE);
  snprintf(tempChar, SMALL_BUFFER, "read ADXL343_REG_INT_ENABLE: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_INT_MAP);
  snprintf(tempChar, SMALL_BUFFER, "read ADXL343_REG_INT_MAP: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_INT_SOURCE);
  snprintf(tempChar, SMALL_BUFFER, "read ADXL343_REG_INT_SOURCE: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_THRESH_ACT);
  snprintf(tempChar, SMALL_BUFFER, "read ADXL343_REG_THRESH_ACT: %i", format);
  Log.info(tempChar);

  format = accel.readRegister(ADXL343_REG_ACT_INACT_CTL);
  snprintf(tempChar, SMALL_BUFFER, "read ADXL343_REG_ACT_INACT_CTL: %i", format);
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
#ifdef USE_ADXL343

void accelerometerInitEnterFunction()
{
  accelerometerSetState(STATE_INIT);
}
void accelerometerInitUpdateFunction()
{

  // stay here a minimum time
  if (accelerometerStateMachine.timeInCurrentState() < INIT_TIMEOUT)
  {
    return;
  }

  accelerometerStateMachine.transitionTo(accelerometerOkState);
  Log.info("Init finished, transition to accelerometerOkState");
}
void accelerometerInitExitFunction()
{
  // clear flag of fired accel events
  g_ints_fired = 0;
  movementDetected = false;
}

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
  // send alarm to ubidots
  sendDataToUbidots(RIGHT_NOW);
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

#endif
