#include <Adafruit_BNO055.h> // For BNO055 IMU
#include <Adafruit_BMP3XX.h> // For BMP388 Altimeter
#include <Servo.h>           // For controlling servo
#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <SPI.h>
#include <LoRa.h>

#define SEALEVELPRESSURE_HPA (1013.25)

uint8_t bigserialbuffer[16384];
const char *filename = "datalog.csv";
File dataFile;
Servo myservo; // servo initialization
float initialh = 0;
int n = 0;
float readings[10];
unsigned long starxbee = 0;
unsigned long starbno = 0;
bool starttransmission = 1;
bool Transmit = 1;
int no_boot = 0;
bool isdescent = 0;

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

byte packet1[2500] = {
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
    0x63,
    0x29,
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

//---------------------------------------- Custom data types sending ------------------------------------------------//
// Define a custom struct to store GNSS data
struct DescentDetector
{
  float previousReading;
  int downwardCount;

  DescentDetector() : previousReading(0), downwardCount(0) {}

  void addReading(float reading)
  {
    Serial.println(downwardCount);
    if (previousReading - reading > 0.5)
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

  data.time = gps.time.value();
  data.speed = gps.speed.mps();
  data.latitude = gps.location.lat();
  data.longitude = gps.location.lng();
  data.altitude = gps.altitude.meters();
  data.satellites = gps.satellites.value();

  return data;
}
//---------------------------------------- GNSS data read types sending end ---------------------------------------------//

//---------------------------------------- Voltage data read types sending ----------------------------------------------//
// Function to read data from the voltage sensor
struct VoltageData readVoltageSensor()
{
  VoltageData data;

  // Read voltage from the sensor (you may need to adjust this based on your sensor)
  data.voltage = (analogRead(A10) * 3.5 * 5) / 1023.0; // Adjust as per your voltage sensor

  return data;
}
//---------------------------------------- Voltage data read types sending end -------------------------------------------//

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
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  data.eulerX = event.orientation.x;
  data.eulerY = event.orientation.y;
  data.eulerZ = event.orientation.z;
  data.accelX = event.acceleration.x;
  data.accelY = event.acceleration.y;
  data.accelZ = event.acceleration.z;
  data.gyroX = gyroz.x();
  data.gyroY = gyroz.y();
  data.gyroZ = gyroz.z();
  data.temperature = bno.getTemp(); // Read temperature from BMP388
  data.magX = meg.x();              // Magnetic field strength X (uT)
  data.magY = meg.y();              // Magnetic field strength Y (uT)
  data.magZ = meg.z();              // Magnetic field strength Z (uT)
  data.linearAccelX = linear.x();   // Linear acceleration X (m/s^2)
  data.linearAccelY = linear.y();   // Linear acceleration Y (m/s^2)
  data.linearAccelZ = linear.z();   // Linear acceleration Z (m/s^2)
  data.gravityX = gravity.x();      // Gravitational acceleration X (m/s^2)
  data.gravityY = gravity.y();      // Gravitational acceleration Y (m/s^2)
  data.gravityZ = gravity.z();      // Gravitational acceleration Z (m/s^2)

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
void check2(unsigned int len)
{

  int sum = 0;
  for (unsigned int i = 3; i < len - 1 + 17; i++)
  {

    sum += packet1[i];
  }
  byte checksum = 0;

  checksum += (sum % 16);
  sum = (sum / 16);
  checksum += (sum % 16) * 16;
  checksum = 255 - checksum;
  packet1[len - 1 + 17] = checksum;
}
//---------------------------------------- checksum calculation for xbee data end ----------------------------------------------------//

String extractAtIndex(const String &input, size_t index)
{
  int startIndex = 0;
  int endIndex = input.indexOf(',');

  // Iterate through the string to find the token at the specified index
  for (size_t i = 0; i < index && endIndex != -1; i++)
  {
    startIndex = endIndex + 1;
    endIndex = input.indexOf(',', startIndex);
  }

  // Extract the token at the specified index
  String result;
  if (endIndex == -1)
  {
    result = input.substring(startIndex);
  }
  else
  {
    result = input.substring(startIndex, endIndex);
  }
  return result;
}

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return;
  String s = "";
  String val = "";
  int valu = 0;
  for (int i = 0; i < packetSize; i++)
  {
    if (i < 2)
    {
      s += (char)LoRa.read();
    }
    else
    {
      val += (char)LoRa.read();
    }
  }
  Serial.print(s);
  valu = val.toInt();

  if (s == "TS")
  {
    starttransmission = 1;
    LoRa.beginPacket();
    LoRa.print("Transmission_started");
    LoRa.endPacket();
    LoRa.receive();
  }

  if (s == "ST")
  {
    starttransmission = 0;
    LoRa.beginPacket();
    LoRa.print("Transmission_stoped");
    LoRa.endPacket();
    LoRa.receive();
  }

  if (s == "RH")
  {

    initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    LoRa.beginPacket();
    LoRa.print("Initialh:-");
    LoRa.print(String(initialh));

    LoRa.endPacket();
    LoRa.receive();
  }

  if (s == "RS")
  {
    for (int pos = 0; pos <= valu; pos += 1) // goes from 0 degrees to 180 degrees
    {                                        // in steps of 1 degree
      myservo.write(pos);                    // tell servo to go to position in variable 'pos'
      delay(30);                             // waits 15ms for the servo to reach the position
    }
    Serial.print(valu);
    LoRa.beginPacket();
    LoRa.print("rotated");
    LoRa.endPacket();
    LoRa.receive();
    delay(50);
  }

  if (s == "CB")
  {

    tone(2, 1000, 2000);
    Serial.print("CB");
    LoRa.beginPacket();
    LoRa.print("buzzer_checked");
    LoRa.endPacket();
    LoRa.receive();
  }

  if (s == "FS")
  {

    Serial.print("FS");
    if (SD.exists(filename))
    {
      SD.remove(filename);

      LoRa.beginPacket();
      LoRa.print("SD_card_formatted\n");
      LoRa.endPacket();
    }
    else
    {

      LoRa.beginPacket();
      LoRa.print("already_formatted\n");
      LoRa.endPacket();
    }

    LoRa.receive();
  }
}

void setup()
{

  tone(2, 1000, 2000);
  // Initialize both serial ports:
  Serial.begin(57600);   // Computer Serial
  Serial7.begin(115200); // GPS connection
  Serial7.addMemoryForRead(&bigserialbuffer, sizeof(bigserialbuffer));
  Serial1.begin(9600); // Xbee connection serial
  myservo.attach(25);
  myservo.write(180); // initial servo position
  // Initialize BNO055 sensor
  if (!bno.begin())
  {
    Serial.println("BNO055 not detected. Please check wiring or I2C address.");
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
  //
  // check for the presence of the SD card:
  if (!SD.begin(BUILTIN_SDCARD))
  { // Change to match your SD card's chip select pin
    Serial.println("SD card initialization failed!");
  }

  if (SD.exists(filename))
  {
    dataFile = SD.open(filename);
    if (dataFile)
    {

      // read from the file until there's nothing else in it:
      Serial.println(dataFile.available());
      long int x = dataFile.available();
      Serial.println(dataFile.position());

      String s = "";
      for (int i = 0; i < 1000; i++)
      {
        dataFile.seek(x - i - 2);

        char temp = dataFile.read();
        if (temp == '\n')
        {
          break;
        }
        s = temp + s;
      }

      String extractedData = extractAtIndex(s, 0);
      Serial.println(s);
      Serial.print("Data at index ");
      Serial.print(0);
      Serial.print(": ");
      Serial.println(extractedData);
      n = extractedData.toInt() + 1;

      extractedData = extractAtIndex(s, 1);
      Serial.print("Data at index ");
      Serial.print(1);
      Serial.print(": ");
      Serial.println(extractedData);
      initialh = extractedData.toInt();

      extractedData = extractAtIndex(s, 2);
      Serial.print("Data at index ");
      Serial.print(2);
      Serial.print(": ");
      Serial.println(extractedData);
      no_boot = extractedData.toInt() + 1;

      extractedData = extractAtIndex(s, 3);
      Serial.print("Data at index ");
      Serial.print(3);
      Serial.print(": ");
      Serial.println(extractedData);
      if (extractedData == "0")
      {
        starttransmission = 0;
      }
      else
      {
        starttransmission = 1;
      }

      extractedData = extractAtIndex(s, 4);
      Serial.print("Data at index ");
      Serial.print(4);
      Serial.print(": ");
      Serial.println(extractedData);

      if (extractedData == "0")
      {
        isdescent = 0;
      }
      else
      {
        isdescent = 0;
      }

      // close the file:
      dataFile.close();
    }
    else
    {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
    }
  }

  // Writing first line
  //  if (dataFile)
  //  {
  //    dataFile.println("TIME_STAMPING,PACKET_COUNT,ALTITUDE,TEMPERATURE,VOLTAGE,GNSS_TIME,PRESSURE,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATS,ACCELEROMETER_X,ACCELEROMETER_Y,ACCELEROMETER_Z,MAGNETOMETER_X,MAGNETOMETER_Y,MAGNETOMETER_Z,CM_ECHO,");
  //    dataFile.close();
  //  }
  //  else
  //  {
  //    Serial.println("Error opening CSV file for writing.");
  //  }

  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setPins(10, 5, 6);
  LoRa.onTxDone(onTxDone);
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive();

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  digitalWrite(SS, LOW);
}

void loop()
{
  if (starttransmission)
  {
    transmit_sequence();
  }
}

void transmit_sequence()
{
  //----------------------------------------  servo control mechanism  -------------------------------------------------//
  if (n == 10)
  {
    initialh = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  if (isdescent && (abs(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialh) < 550))
  {
    // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(0); // tell servo to go to position in variable 'pos'
    delay(20);
    Serial.println("gfdf"); // waits 15ms for the servo to reach the position
  }
  //----------------------------------------  servo control mechanism end  ---------------------------------------------//
  //----------------------------------------  data Parse for GPS  -------------------------------------------------//
  unsigned long starttime = millis();
  while ((millis() - starttime) <= 10) // do this loop for up to 1000mS
  {
    //    Serial.print(n);
    if (Serial7.available())
    {
      //      Serial.println(Serial7.available());
      char c = Serial7.read();
      ////    Serial.write(c); // Uncomment this line if you want to see the GPS data flowing
      gps.encode(c);
      //      Serial.print(c);
    }
  }
  //----------------------------------------  data Parse for GPS end  ----------------------------------------------//

  //----------------------------------------  data read for GPS,BNO,etc  -------------------------------------------------//

  struct GNSSData gnssData = readGNSSData(gps);
  struct VoltageData voltageData = readVoltageSensor();

  struct BNO055Data bnoData = readBNO055Sensor();
  //----------------------------------------  data read for GPS,BNO,etc ends ---------------------------------------------//
  // Now, you have the GNSS data in gnssData, the voltage data in voltageData,
  // the vibration data in vibrationData, and the BNO055 sensor data in bnoData structs.
  // You can use these data as needed or store them in variables.
  // For example:

  //---------------------------------------- Making string of data for transmission procedure ---------------------------------------------//
  String data = ""; // string containing the data

  float voltage = voltageData.voltage;

  data += String(n) + ",";                                                    // 0
  data += String(initialh) + ",";                                             // 1
  data += String(no_boot) + ",";                                              // 2
  data += String(starttransmission) + ",";                                    // 3
  data += String(isdescent) + ",";                                            // 4
  data += String(gnssData.time) + ",";                                        // 5
  data += String(gnssData.speed, 2) + ",";                                    // 6
  data += String(gnssData.latitude, 7) + ",";                                 // 7
  data += String(gnssData.longitude, 7) + ",";                                // 8
  data += String(gnssData.altitude, 2) + ",";                                 // 9
  data += String(gnssData.satellites, 2) + ",";                               // 10
  data += String(voltage, 2) + ",";                                           // 11
  data += String(bnoData.eulerX, 2) + ",";                                    // 12
  data += String(bnoData.eulerY, 2) + ",";                                    // 13
  data += String(bnoData.eulerZ, 2) + ",";                                    // 14
  data += String(bnoData.accelX, 2) + ",";                                    // 15
  data += String(bnoData.accelY, 2) + ",";                                    // 16
  data += String(bnoData.accelZ, 2) + ",";                                    // 17
  data += String(bnoData.gyroX, 2) + ",";                                     // 18
  data += String(bnoData.gyroY, 2) + ",";                                     // 19
  data += String(bnoData.gyroZ, 2) + ",";                                     // 20
  data += String(bnoData.temperature, 2) + ",";                               // 21
  data += String(bnoData.magX, 2) + ",";                                      // 22
  data += String(bnoData.magY, 2) + ",";                                      // 23
  data += String(bnoData.magZ, 2) + ",";                                      // 24
  data += String(bnoData.linearAccelX, 2) + ",";                              // 25
  data += String(bnoData.linearAccelY, 2) + ",";                              // 26
  data += String(bnoData.linearAccelZ, 2) + ",";                              // 27
  data += String(bnoData.gravityX, 2) + ",";                                  // 28
  data += String(bnoData.gravityY, 2) + ",";                                  // 29
  data += String(bnoData.gravityZ, 2) + ",";                                  // 30
  data += String(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialh, 2) + ","; // 31
  data += String(bmp.temperature, 2) + ",";                                   // 32
  data += String(bmp.pressure / 100.0, 2) + ",";                              // 33

  //  Serial.println(data.length());
  //  Serial.println(initialh);

  //---------------------------------------- Making string of data for transmission procedure ends -----------------------------------------//

  //---------------------------------------- Xbee data transmission procedure starts ---------------------------------------------//
  byte datasend[2500];
  data.getBytes(datasend, data.length());
  for (unsigned int i = 0; i < data.length() - 1; i++)
  {
    packet[17 + i] = datasend[i];
    packet1[17 + i] = datasend[i];
  }
  // length of data start
  packet[1] = (14 + data.length() - 1) / 256;
  packet[2] = (14 + data.length() - 1) % 256;
  packet1[1] = (14 + data.length() - 1) / 256;
  packet1[2] = (14 + data.length() - 1) % 256;
  // length of data end
  check(data.length());
  check2(data.length());
  //  for (unsigned int i =  0 ; i< data.length()+17 ; i++){
  //    Serial.print(packet[i]);
  //    Serial.print(" ");
  //  }
  //  Serial.println();

  if (millis() - starxbee > 500)
  {
    Serial1.write(packet, data.length() + 17);
    delay(100);
    Serial1.write(packet1, data.length() + 17);
    starxbee = millis();
    if ((Transmit == 1) && (bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialh >= 20))
    {
      LoRa.beginPacket();
      LoRa.print(data);
      LoRa.endPacket(true);
      Transmit = 0;
    }
    n++;
    Serial.println(data);
  }

  //---------------------------------------- Xbee data transmission procedure ends ---------------------------------------------//

  //---------------------------------------- Sd card save data transmission procedure starts ---------------------------------------------//
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(data);
    dataFile.close();
  }
  else
  {
    Serial.println("Error opening CSV file for writing.");
    //    SD.begin(BUILTIN_SDCARD);
  }

  //---------------------------------------- Sd card save data transmission procedure ends   ---------------------------------------------//4

  //---------------------------------------- Descent detection starts ---------------------------------------------//

  if (!isdescent)
  {

    if (abs(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialh) > 600)
    {
      isdescent = 1;
    }
  }

  if (isdescent && (abs(bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialh) < 100))
  {

    tone(2, 1000, 2000);
  }

  //---------------------------------------- Descent detection ends ---------------------------------------------//

  // You can perform further actions with the data here

  // Adjust the delay as needed (Currently capturing data as quickly as possible)
}
void onTxDone()
{
  Serial.println("Done");
  Transmit = 1;
}
