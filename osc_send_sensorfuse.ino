// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface.

// Sends all data via OSC for communicating with other devices.

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_DPS310.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

char ssid[] = "Ale";          // your network SSID (name)
char pass[] = "alexela5";                    // your network password
//
//char ssid[] = "TP-Link_1E86";          // your network SSID (name)
//char pass[] = "98204564";                    // your network password
//


WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
//const IPAddress outIp(192, 168, 1, 45);     // remote IP of your computer
const IPAddress outIp(192, 168, 43, 154);     // remote IP of your computer
const unsigned int outPort = 8001;          // remote port to receive OSC
const unsigned int localPort = 9001;        // local port to listen for OSC packets (actually not used for sending)

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();


// Can also use SPI!
#define DPS310_CS 10

// uncomment one combo 9-DoF!
//#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
  
//// THIS SETS THE PRESSURE SENSOR
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
  //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1) yield();
  }
  Serial.println("DPS OK!");

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  dps_temp->printSensorDetails();
  dps_pressure->printSensorDetails();

///// THIS SETS UP THE WIFI UDP OSC CONNECTION
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  //Serial.println(Udp.localPort());
}


void loop() {
 
  ///read temp and pressure. prints on console.  
  dps310_read(); 

  ///// NXP FUSION 
  nxp_read();
  
}

void dps310_read(){
  sensors_event_t temp_event, pressure_event;

  /// DPS PRESSURE 
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    Serial.println();
    OSCMessage msgT("/temp");
    msgT.add(temp_event.temperature);
    Udp.beginPacket(outIp, outPort);
    msgT.send(Udp);
    Udp.endPacket();
    msgT.empty();

    
  }

  // Reading pressure also reads temp so don't check pressure
  // before temp!
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa"); 
    Serial.println();
    OSCMessage msgP("/pressure");
    msgP.add(pressure_event.pressure);
    Udp.beginPacket(outIp, outPort);
    msgP.send(Udp);
    Udp.endPacket();
    msgP.empty();
  }
}

void nxp_read(){
  
  float roll, pitch, heading;
  float gx, gy, gz;
  static uint8_t counter = 0;
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

  //// prints raw Accelerometer data and sends via UPD 
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  //UDP
  OSCMessage msgA("/accel");
  msgA.add(accel.acceleration.x);
  msgA.add(accel.acceleration.y);
  msgA.add(accel.acceleration.z);
  Udp.beginPacket(outIp, outPort);
  msgA.send(Udp);
  Udp.endPacket();
  msgA.empty();

  //// prints raw Gyroscope data and sends via UPD 
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  //UDP
  OSCMessage msgG("/gyro");
  msgG.add(gx);
  msgG.add(gy);
  msgG.add(gz);
  Udp.beginPacket(outIp, outPort);
  msgG.send(Udp);
  Udp.endPacket();
  msgG.empty();

  
  //// prints raw Magnetometer data and sends via UPD 
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
  //UDP
  OSCMessage msgM("/mag");
  msgM.add(mag.magnetic.x);
  msgM.add(mag.magnetic.y);
  msgM.add(mag.magnetic.z);
  Udp.beginPacket(outIp, outPort);
  msgM.send(Udp);
  Udp.endPacket();
  msgM.empty();
  
  // print the heading, pitch and roll. Euler angles as Orientation. UDP send
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);

  OSCMessage msgO("/Orientation");
  msgO.add(heading);
  msgO.add(pitch);
  msgO.add(roll);
  Udp.beginPacket(outIp, outPort);
  msgO.send(Udp);
  Udp.endPacket();
  msgO.empty();
  
  // print the Quaternions. UDP send
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4);  

  OSCMessage msgQ("/Quaternion");
  msgQ.add(qw);
  msgQ.add(qx);
  msgQ.add(qy);
  msgQ.add(qz);
  Udp.beginPacket(outIp, outPort);
  msgQ.send(Udp);
  Udp.endPacket();
  msgQ.empty();
}
