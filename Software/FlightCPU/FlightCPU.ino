//Common
#include "Wire.h"
#include "I2Cdev.h"

/********************
    Pressure
*/
#include <SFE_BMP180.h>
// You will need to create an SFE_BMP180 object, here called "pressure":
SFE_BMP180 pressure;
double baselineP; // baseline pressure
void pressureFunc();
void getTempPressure(double &T, double &P);


/********************
    Compass-Magmometer
*/

#include "HMC5883L.h"
// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;
void compassFunc();



/********************
    Acceler-Gyro
*/
#include "MPU6050.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

void accelGyroFunc();
void accelGyroFactoryReset();


/********************
   Humidity/Temp
*/
void humdTempFunc();


/********************
   SD card datalogger
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
*/
#include <SPI.h>
#include <SD.h>
const int chipSelect = 4;
File dataFile;
void initDataFile();
void writeToSD(String dataLine);
void writeToSD(double dataLine);
void writeToSDln(double dataLine);
void writeToSDln(String dataLine);


/********************
    IO
*/
#define LED_PIN 0
bool blinkState = false;
unsigned long t;


void setup() {
  // Initialize the LED- Must be first for data indicators
  pinMode(LED_PIN, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  //Initialize SD Card
  initDataFile();

  // initialize device
  mag.initialize();
  accelgyro.initialize();

  // verify connection
  writeToSDln(mag.testConnection() ? F("HMC5883L connection successful") : F("HMC5883L connection failed"));      //Compass
  writeToSDln(pressure.begin() ? F("BMP180 connection successful") : F("BMP180 connection failed"));              //Barometer
  writeToSDln(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  //AccellGyro

  //Calibrate any baselines
  // Get the baseline pressure:
  double T;// through away temp
  getTempPressure(T,baselineP);
}

void loop() {
  //Log time
  t = millis();
  writeToSD("t,");
  writeToSDln(t);
  
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  compassFunc();
  delay(250);
  
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  accelGyroFunc();
  delay(250);
  

  
  humdTempFunc();
  delay(500);
  

  compassFunc();
  delay(250);
  accelGyroFunc();
  delay(250);

  pressureFunc();
  delay(500);
}

//C,mx,my,mx,h
//Compass,magX,magY,magZ,heading
void compassFunc() {
  writeToSD(F("C,"));
  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);

  // display tab-separated gyro x/y/z values
  writeToSD(mx); writeToSD(",");
  writeToSD(my); writeToSD(F(","));
  writeToSD(mz); writeToSD(F(","));

  // To calculate heading in degrees. 0 degree indicates North
  float heading = atan2(my, mx);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  writeToSDln(heading * 180 / M_PI);
}

//B,t,p,a
//Barameter,Temp(f),Pressure(mb),Altitude(ft)
void pressureFunc() {
  double T, P, A;
  writeToSD(F("B,"));

  getTempPressure(T, P);

  //We now have Pessure, Temp.. Go Calculate Altitude
  //Pressure
  writeToSD(P, 2);
  //Temp
  writeToSD(F(","));
  writeToSD((9.0 / 5.0)*T + 32.0, 2);
  //Altitude
  writeToSD(F(","));
  A = pressure.altitude(P, baselineP);
  writeToSDln(A * 3.28084, 0); //Altitude in ft
}
void getTempPressure(double &T, double &P) {
  char status;
  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {



        }
        else writeToSDln(F("e1")); //error retrieving pressure
      }
      else writeToSDln(F("e2")); //error starting pressure
    }
    else writeToSDln(F("e3")); //error retrieving temp
  }
  else writeToSDln(F("e4")); //error starting temp
}
//ax,ay,az,gz,gx,gy
void accelGyroFunc() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  writeToSD("a/g,");
  writeToSD(ax); writeToSD(F(","));
  writeToSD(ay); writeToSD(F(","));
  writeToSD(az); writeToSD(F(","));
  writeToSD(gx); writeToSD(F(","));
  writeToSD(gy); writeToSD(F(","));
  writeToSDln(gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
#endif

}
//S,t,h
//Si7020,Temp(f),Humidity(%,relative)
void humdTempFunc() {
  writeToSD(F("S,"));

  int index = 0;
  word data = 0;
  byte readData [20];

  //**************Request Temp
  Wire.beginTransmission(0x40);
  Wire.write(0xE3); //Hold Master mode- no hold doesn't seem to work
  Wire.endTransmission();
  //Read the value back
  Wire.requestFrom(0x40, 2);
  while (Wire.available()) {
    readData[index++] = Wire.read();
  }
  //Process the Data
  data = readData[0] << 8;
  data = data | readData[1];
  float tempC;
  tempC = (175.72 * data) / 65536 - 46.85;
  float tempF;
  tempF = (tempC * 9) / 5 + 32.0;
  writeToSD(tempF, 4);
  writeToSD(F(","));
  delay(30); //Arbitrary delay between commands


  //**************Read Humid
  index = 0;
  Wire.beginTransmission(0x40);
  Wire.write(0xE5); //Hold Master mode- no hold doesn't seem to work
  Wire.endTransmission();
  //Read the value back
  Wire.requestFrom(0x40, 2);
  while (Wire.available()) {
    readData[index++] = Wire.read();
  }
  //Process the Data
  data = readData[0] << 8;
  data = data | readData[1];
  float relativeHumid;
  relativeHumid = (125.0 * data) / 65536 - 6.0;
  writeToSDln(relativeHumid, 2);
}
/**************************************************************************
   SD CARD
*/
void initDataFile() {
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //If we could not initialize, blink LED
    while (1) {
      delay(250);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
    return;
  }
}


void writeToSD(double data) {
  File dFile;
  dFile = SD.open(F("COPE01.txt"), FILE_WRITE);

  if (dFile) {
    dFile.print(data);
    dFile.close();
  }
  else {
    for (int i = 0; i < 4; i++) {
      delay(500);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}
void writeToSD(double data, int percision) {
  File dFile;
  dFile = SD.open(F("COPE01.txt"), FILE_WRITE);

  if (dFile) {
    dFile.print(data, percision);
    dFile.close();
  }
  else {
    for (int i = 0; i < 4; i++) {
      delay(500);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}
void writeToSD(String line) {
  File dFile;
  dFile = SD.open(F("COPE01.txt"), FILE_WRITE);

  if (dFile) {
    dFile.print(line);
    dFile.close();
  }
  else {
    for (int i = 0; i < 4; i++) {
      delay(500);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}

/******************* New Line ***********

*/

void writeToSDln(double data) {
  File dFile;
  dFile = SD.open(F("COPE01.txt"), FILE_WRITE);

  if (dFile) {
    dFile.println(data);
    dFile.close();
  }
  else {
    for (int i = 0; i < 4; i++) {
      delay(500);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}
void writeToSDln(double data, int percision) {
  File dFile;
  dFile = SD.open(F("COPE01.txt"), FILE_WRITE);

  if (dFile) {
    dFile.println(data, percision);
    dFile.close();
  }
  else {
    for (int i = 0; i < 4; i++) {
      delay(500);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}
void writeToSDln(String line) {
  File dFile;
  dFile = SD.open(F("COPE01.txt"), FILE_WRITE);

  if (dFile) {
    dFile.println(line);
    dFile.close();
  }
  else {
    for (int i = 0; i < 4; i++) {
      delay(500);
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }
}


