#include <TimeLib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

// Demo for getting individual unified sensor data from the ICM20649
#include <Adafruit_ICM20649.h>

Adafruit_ICM20649 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;

#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

int led = 13;
File flight_data;

void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while (!Serial){
    delay(10);
  }
  //IMU test
  Serial.println("Adafruit ICM20649 test!");
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20649 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20649 Found!");
  icm_temp = icm.getTemperatureSensor();
  icm_temp->printSensorDetails();

  icm_accel = icm.getAccelerometerSensor();
  icm_accel->printSensorDetails();

  icm_gyro = icm.getGyroSensor();
  icm_gyro->printSensorDetails();
  //Altimeter Test
  Serial.println("Adafruit BMP388 / BMP390 test");
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  
  // initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Initialization failed");
    return;
  }
  Serial.println("Initialization complete");
}


unsigned long runtime = 60000;

// the loop routine runs over and over again forever:
void loop() {
  if (millis() < runtime) {
    flight_data = SD.open("flight_data.txt", FILE_WRITE);

    if (flight_data) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(2);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(2);
      // wait

      //IMU display
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      icm_temp->getEvent(&temp);
      icm_accel->getEvent(&accel);
      icm_gyro->getEvent(&gyro);

      flight_data.print("Time elapsed: ");
      flight_data.print(millis());
      flight_data.println(" ms");
      
      /* Display the results (temperature is measured in deg C) */
      Serial.print("Temperature = ");
      Serial.print(temp.temperature);
      Serial.println(" deg C");

      // Write temperature to SD card
      flight_data.print("Temperature ");
      flight_data.print(temp.temperature);
      flight_data.println(" deg C");
      
      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print("Accel X: ");
      Serial.print(accel.acceleration.x);
      Serial.print("\t\tY: ");
      Serial.print(accel.acceleration.y);
      Serial.print("\t\tZ: ");
      Serial.print(accel.acceleration.z);
      Serial.println(" m/s^2 ");

      // Write acceleration to SD card
      flight_data.print("Accel X: ");
      flight_data.print(accel.acceleration.x);
      flight_data.print("\t\tY: ");
      flight_data.print(accel.acceleration.y);
      flight_data.print("\t\tZ: ");
      flight_data.print(accel.acceleration.z);
      flight_data.println(" m/s^2 ");

      /* Display the results (rotation is measured in rad/s) */
      Serial.print("Gyro X: ");
      Serial.print(gyro.gyro.x);
      Serial.print("\t\tY: ");
      Serial.print(gyro.gyro.y);
      Serial.print("\t\tZ: ");
      Serial.print(gyro.gyro.z);
      Serial.println(" radians/s ");

      
      // Write gyro to SD card
      flight_data.print("Gyro X: ");
      flight_data.print(gyro.gyro.x);
      flight_data.print("\t\tY: ");
      flight_data.print(gyro.gyro.y);
      flight_data.print("\t\tZ: ");
      flight_data.print(gyro.gyro.z);
      flight_data.println(" radians/s ");


      if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }

      /* Display the results (pressure is measured in hPa)*/
      Serial.print("Pressure = ");
      Serial.print(bmp.pressure / 100.0);
      Serial.println(" hPa");

      // Write pressure to SD card
      flight_data.print("Pressure = ");
      flight_data.print(bmp.pressure / 100.0);
      flight_data.println(" hPa");


      /* Display the results (altitude is measured in m)*/
      Serial.print("Approx. Altitude = ");
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
      Serial.println();

      //Write altitude to SD card
      flight_data.print("Approx. Altitude = ");
      flight_data.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      flight_data.println(" m");
      flight_data.println();

      flight_data.close();
    } else {
      Serial.println("Error opening file in loop.");
    }

    delay(2);

  } else {
    Serial.print("60 seconds elapsed");
    exit(0);
  }
}
