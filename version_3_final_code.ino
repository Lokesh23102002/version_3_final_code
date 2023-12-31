#include <Adafruit_BNO055.h> // For BNO055 IMU
#include <Adafruit_BMP3XX.h> // For BMP388 Altimeter
#include <Servo.h>           // For controlling servo
#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>

const char *filename = "datalog.csv";
File dataFile;
Servo myservo; // servo initialization
float initialh = 0;
int n = 0;
float readings[10];
unsigned long starxbee = 0;
unsigned long starbno = 0;
#define SEALEVELPRESSURE_HPA (1013.25)
// first 17 bits of xbee data with address start delimeter etc
byte packet[2500] = {
    0x7E,
    0x00,
    0x0E,
    0x10,
    0x01,
    0x00,
    0x13,
    0xA2,
    0x00,
    0x41,
    0xF4,
    0x25,
    0xE8,
    0xFF,
    0xFE,
    0x00,
    0x00,
};
// end
// Tiny gps for gps data parsing
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO sensor data
Adafruit_BMP3XX bmp;                             // BMP library initialization

int D2 = 5;
//---------------------------------------- Custom data types sending ------------------------------------------------//
// Define a custom struct to store GNSS data

struct DescentDetector
{
  float previousReading;
  int downwardCount;

  DescentDetector() : previousReading(0), downwardCount(0) {}

  void addReading(float reading)
  {
    if (reading < previousReading)
    {
      downwardCount++;
    }
    previousReading = reading;
  }

  bool detectDownwardDescent()
  {
    // Return true if there are at least 7 downward descents
    return downwardCount > 7;
  }
};

struct DescentDetector descentDetector;

struct GNSSData
{
  long time;
  float speed;
  double latitude;
  double longitude;
  float altitude;
  int satellites;
};

// Define a custom struct to store voltage sensor data
struct VoltageData
{
  float voltage;
};

// Define a custom struct to store vibration sensor data
struct VibrationData
{
  int vibration;
};

// Define a custom struct to store BNO055 sensor data (9 DOF)
struct BNO055Data
{
  float eulerX;
  float eulerY;
  float eulerZ;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temperature;  // Temperature in degrees Celsius
  float magX;         // Magnetic field strength X (uT)
  float magY;         // Magnetic field strength Y (uT)
  float magZ;         // Magnetic field strength Z (uT)
  float linearAccelX; // Linear acceleration X (m/s^2)
  float linearAccelY; // Linear acceleration Y (m/s^2)
  float linearAccelZ; // Linear acceleration Z (m/s^2)
  float gravityX;     // Gravitational acceleration X (m/s^2)
  float gravityY;     // Gravitational acceleration Y (m/s^2)
  float gravityZ;     // Gravitational acceleration Z (m/s^2)
};
//---------------------------------------- Custom data types sending end ---------------------------------------------//

//---------------------------------------- GNSS data read types sending  ------------------------------------------------//
// Function to read and parse data from the GNSS sensor
struct GNSSData readGNSSData(TinyGPSPlus &gps)
{
  GNSSData data;

  // Check if GPS data is available
  if (gps.location.isUpdated())
  {
    data.time = gps.time.value();
    data.speed = gps.speed.mps();
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.meters();
    data.satellites = gps.satellites.value();
  }
  else
  {
    // Set default values if data is not available
    data.time = 0;
    data.speed = 0.0;
    data.latitude = 0.0;
    data.longitude = 0.0;
    data.altitude = 0.0;
    data.satellites = 0;
  }

  return data;
}
//---------------------------------------- GNSS data read types sending end ---------------------------------------------//

//---------------------------------------- Voltage data read types sending ----------------------------------------------//
// Function to read data from the voltage sensor
struct VoltageData readVoltageSensor()
{
  VoltageData data;

  // Read voltage from the sensor (you may need to adjust this based on your sensor)
  data.voltage = (analogRead(A0) * 3.3 * 5) / 1023.0; // Adjust as per your voltage sensor

  return data;
}
//---------------------------------------- Voltage data read types sending end -------------------------------------------//

//---------------------------------------- Vibration data read types sending ---------------------------------------------//
// Function to read data from the SW420 vibration sensor
struct VibrationData readVibrationSensor()
{
  VibrationData data;

  // Read vibration sensor data (you may need to adjust this based on your sensor)
  data.vibration = digitalRead(D2); // Adjust as per your vibration sensor

  return data;
}
//---------------------------------------- vibration data read types sending end -------------------------------------------//

//---------------------------------------- BNO data read types sending  ----------------------------------------------------//
// Function to read data from the BNO055 sensor (9 DOF)
struct BNO055Data readBNO055Sensor()
{
  sensors_event_t event;
  BNO055Data data;

  bno.getEvent(&event);
  imu::Vector<3> meg = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroz = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data.eulerX = event.orientation.x;
  data.eulerY = event.orientation.y;
  data.eulerZ = event.orientation.z;
  data.accelX = event.acceleration.x;
  data.accelY = event.acceleration.y;
  data.accelZ = event.acceleration.z;
  data.gyroX = gyroz.x();
  data.gyroY = gyroz.y();
  data.gyroZ = gyroz.z();
  data.temperature = bno.getTemp();     // Read temperature from BMP388
  data.magX = meg.x();                  // Magnetic field strength X (uT)
  data.magY = meg.y();                  // Magnetic field strength Y (uT)
  data.magZ = meg.z();                  // Magnetic field strength Z (uT)
  data.linearAccelX = acc.x();          // Linear acceleration X (m/s^2)
  data.linearAccelY = acc.y();          // Linear acceleration Y (m/s^2)
  data.linearAccelZ = acc.z();          // Linear acceleration Z (m/s^2)
  data.gravityX = event.acceleration.x; // Gravitational acceleration X (m/s^2)
  data.gravityY = event.acceleration.y; // Gravitational acceleration Y (m/s^2)
  data.gravityZ = event.acceleration.z; // Gravitational acceleration Z (m/s^2)

  return data;
}
//---------------------------------------- BNO data read types sending end -------------------------------------------//

//---------------------------------------- checksum calculation for xbee data sending ------------------------------------------------//
void check(unsigned int len)
{

  int sum = 0;
  for (unsigned int i = 3; i < len - 1 + 17; i++)
  {

    sum += packet[i];
  }
  byte checksum = 0;

  checksum += (sum % 16);
  sum = (sum / 16);
  checksum += (sum % 16) * 16;
  checksum = 255 - checksum;
  packet[len - 1 + 17] = checksum;
}
//---------------------------------------- checksum calculation for xbee data end ----------------------------------------------------//

void setup()
{
  // Initialize both serial ports:
  Serial.begin(57600);
  Serial1.begin(9600); // GPS connection
  Serial2.begin(9600); // Xbee connection serial
  myservo.attach(5);
  myservo.write(180); // initial servo position
  // Initialize BNO055 sensor
  if (!bno.begin())
  {
    Serial.println("BNO055 not detected. Please check wiring or I2C address.");
    while (1)
      ;
  }
  starxbee = millis();
  starbno = millis();

  // Initialize BMP388 sensor
  if (!bmp.begin_I2C(0x76, &Wire1))
  {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  // check for the presence of the SD card:
  if (!SD.begin(BUILTIN_SDCARD))
  { // Change to match your SD card's chip select pin
    Serial.println("SD card initialization failed!");
    while (1)
      ;
  }

  dataFile = SD.open(filename, FILE_WRITE);

  // Writing first line
  if (dataFile)
  {
    dataFile.print("TIME_STAMPING,PACKET_COUNT,ALTITUDE,TEMPERATURE,VOLTAGE,GNSS_TIME,PRESSURE,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATS,ACCELEROMETER_X,ACCELEROMETER_Y,ACCELEROMETER_Z,MAGNETOMETER_X,MAGNETOMETER_Y,MAGNETOMETER_Z,CM_ECHO,");
    dataFile.close();
  }
  else
  {
    Serial.println("Error opening CSV file for writing.");
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println(initialh);
  initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.println(initialh);
}

void loop()
{
  //----------------------------------------  servo control mechanism  -------------------------------------------------//
  if (n == 0)
  {
    initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  if (initialh - bmp.readAltitude(SEALEVELPRESSURE_HPA) > 5)
  {
    myservo.write(0);
    delay(50);
  }
  //----------------------------------------  servo control mechanism end  ---------------------------------------------//
  //----------------------------------------  data Parse for GPS  -------------------------------------------------//
  unsigned long starttime = millis();
  while ((millis() - starttime) <= 100) // do this loop for up to 1000mS
  {
    if (Serial1.available())
    {
      char c = Serial1.read();
      ////    Serial.write(c); // Uncomment this line if you want to see the GPS data flowing
      gps.encode(c);
    }
  }
  //----------------------------------------  data Parse for GPS end  ----------------------------------------------//

  //----------------------------------------  data read for GPS,BNO,etc  -------------------------------------------------//
  struct GNSSData gnssData = readGNSSData(gps);
  struct VoltageData voltageData = readVoltageSensor();
  struct VibrationData vibrationData = readVibrationSensor();

  struct BNO055Data bnoData = readBNO055Sensor();
  //----------------------------------------  data read for GPS,BNO,etc ends ---------------------------------------------//
  // Now, you have the GNSS data in gnssData, the voltage data in voltageData,
  // the vibration data in vibrationData, and the BNO055 sensor data in bnoData structs.
  // You can use these data as needed or store them in variables.
  // For example:

  //---------------------------------------- Making string of data for transmission procedure ---------------------------------------------//
  String data = ""; // string containing the data

  float voltage = voltageData.voltage;
  int vibration = vibrationData.vibration;

  data += String(gnssData.time) + ",";
  data += String(n) + ",";
  data += String(gnssData.speed, 2) + ",";
  data += String(gnssData.latitude, 7) + ",";
  data += String(gnssData.longitude, 7) + ",";
  data += String(gnssData.altitude, 2) + ",";
  data += String(gnssData.satellites, 2) + ",";
  data += String(voltage, 2) + ",";
  data += String(vibration, 2) + ",";
  data += String(bnoData.eulerX, 2) + ",";
  data += String(bnoData.eulerY, 2) + ",";
  data += String(bnoData.eulerZ, 2) + ",";
  data += String(bnoData.accelX, 2) + ",";
  data += String(bnoData.accelY, 2) + ",";
  data += String(bnoData.accelZ, 2) + ",";
  data += String(bnoData.gyroX, 2) + ",";
  data += String(bnoData.gyroY, 2) + ",";
  data += String(bnoData.gyroZ, 2) + ",";
  data += String(bnoData.temperature, 2) + ",";
  data += String(bnoData.magX, 2) + ",";
  data += String(bnoData.magY, 2) + ",";
  data += String(bnoData.magZ, 2) + ",";
  data += String(bnoData.linearAccelX, 2) + ",";
  data += String(bnoData.linearAccelY, 2) + ",";
  data += String(bnoData.linearAccelZ, 2) + ",";
  data += String(bnoData.gravityX, 2) + ",";
  data += String(bnoData.gravityY, 2) + ",";
  data += String(bnoData.gravityZ, 2) + ",";
  //  data += String(initialh-bmp.readAltitude(SEALEVELPRESSURE_HPA), 2) + ",";
  //  data += String(initialh-bmp.readAltitude(SEALEVELPRESSURE_HPA), 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  //  data += String(0, 2) + ",";
  data += String(bmp.readAltitude(SEALEVELPRESSURE_HPA), 2) + ",";
  data += String(bmp.temperature, 2) + ",";
  data += String(bmp.pressure / 100.0, 2) + ",";

  Serial.println(data);
  Serial.println(data.length());
  Serial.println(initialh);

  //---------------------------------------- Making string of data for transmission procedure ends -----------------------------------------//

  //---------------------------------------- Xbee data transmission procedure starts ---------------------------------------------//
  byte datasend[2500];
  data.getBytes(datasend, data.length());
  for (unsigned int i = 0; i < data.length() - 1; i++)
  {
    packet[17 + i] = datasend[i];
  }
  // length of data start
  packet[1] = (14 + data.length() - 1) / 256;
  packet[2] = (14 + data.length() - 1) % 256;
  // length of data end
  check(data.length());
  //  for (unsigned int i =  0 ; i< data.length()+17 ; i++){
  //    Serial.print(packet[i]);
  //    Serial.print(" ");
  //  }
  //  Serial.println();
  if (millis() - starxbee > 100)
  {
    Serial2.write(packet, data.length() + 17);
    starxbee = millis();
    n++;
  }
  //---------------------------------------- Xbee data transmission procedure ends ---------------------------------------------//

  //---------------------------------------- Sd card save data transmission procedure starts ---------------------------------------------//
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile)
  {
    dataFile.print(data);
    dataFile.close();
  }
  else
  {
    Serial.println("Error opening CSV file for writing.");
  }

  //---------------------------------------- Sd card save data transmission procedure ends   ---------------------------------------------//4

  //---------------------------------------- Descent detection starts ---------------------------------------------//

  descentDetector.addReading(bmp.readAltitude(SEALEVELPRESSURE_HPA));

  //---------------------------------------- Descent detection ends ---------------------------------------------//

  // You can perform further actions with the data here

  // Adjust the delay as needed (Currently capturing data as quickly as possible)
}
