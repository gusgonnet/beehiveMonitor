#include "Adafruit_Sensor.h"

// SerialLogHandler logHandler(LOG_LEVEL_ALL);

/**************************************************************************/
/*!
    @brief  Prints sensor information to serial console
*/
/**************************************************************************/
void Adafruit_Sensor::printSensorDetails(void) {
  sensor_t sensor;
  getSensor(&sensor);
  // Log.info("------------------------------------");
  // Log.info("Sensor:       ");
  // Log.info(sensor.name);
  // Log.info("Type:         ");
  // switch ((sensors_type_t)sensor.type) {
  // case SENSOR_TYPE_ACCELEROMETER:
  //   Log.info("Acceleration (m/s2)");
  //   break;
  // case SENSOR_TYPE_MAGNETIC_FIELD:
  //   Log.info("Magnetic (uT)");
  //   break;
  // case SENSOR_TYPE_ORIENTATION:
  //   Log.info("Orientation (degrees)");
  //   break;
  // case SENSOR_TYPE_GYROSCOPE:
  //   Log.info("Gyroscopic (rad/s)");
  //   break;
  // case SENSOR_TYPE_LIGHT:
  //   Log.info("Light (lux)");
  //   break;
  // case SENSOR_TYPE_PRESSURE:
  //   Log.info("Pressure (hPa)");
  //   break;
  // case SENSOR_TYPE_PROXIMITY:
  //   Log.info("Distance (cm)");
  //   break;
  // case SENSOR_TYPE_GRAVITY:
  //   Log.info("Gravity (m/s2)");
  //   break;
  // case SENSOR_TYPE_LINEAR_ACCELERATION:
  //   Log.info("Linear Acceleration (m/s2)");
  //   break;
  // case SENSOR_TYPE_ROTATION_VECTOR:
  //   Log.info("Rotation vector");
  //   break;
  // case SENSOR_TYPE_RELATIVE_HUMIDITY:
  //   Log.info("Relative Humidity (%)");
  //   break;
  // case SENSOR_TYPE_AMBIENT_TEMPERATURE:
  //   Log.info("Ambient Temp (C)");
  //   break;
  // case SENSOR_TYPE_OBJECT_TEMPERATURE:
  //   Log.info("Object Temp (C)");
  //   break;
  // case SENSOR_TYPE_VOLTAGE:
  //   Log.info("Voltage (V)");
  //   break;
  // case SENSOR_TYPE_CURRENT:
  //   Log.info("Current (mA)");
  //   break;
  // case SENSOR_TYPE_COLOR:
  //   Log.info("Color (RGBA)");
  //   break;
  // }

  // Log.info();
  // Log.info("Driver Ver:   ");
  // Log.info(sensor.version);
  // Log.info("Unique ID:    ");
  // Log.info(sensor.sensor_id);
  // Log.info("Min Value:    ");
  // Log.info(sensor.min_value);
  // Log.info("Max Value:    ");
  // Log.info(sensor.max_value);
  // Log.info("Resolution:   ");
  // Log.info(sensor.resolution);
  // Log.info("------------------------------------\n");
}
