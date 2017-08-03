#include <imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


/* > General Declarations < */
const bool PRINT_MEASUREMENTS = false;
const bool WRITE_DEMO_FILE    = false;
const int sampling_period = 50;      // samping period of sensors
int time_step             = 0;

/* > Datalogger Declarations < */
const String HEADER = "time, th_x, th_y, th_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, force, temperature";
const String CSV_name = "0803_001.csv";
const int CSpin = 4;
File dataFile;

/* > Orientation Sensor Declarations < */
Adafruit_BNO055 bno = Adafruit_BNO055(55); // What does the 55 do?
imu::Vector<3> orientation;
imu::Vector<3> acceleration;
imu::Vector<3> magnetic_field;
const int ACCELERATION_SF = 100;
int temperature;

/* > Force Sensor Declarations < */
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;
int force;


void setup(void) {
  Serial.begin(9600);
 
  /* 1. Initialize the SD card. */
  Serial.print("Initializing SD card... ");
  pinMode(CSpin, OUTPUT);
 
  if (!SD.begin(CSpin)) {
    Serial.println("Card failed, or not present.");
    return;
  }
  else {
    Serial.println("Card initialized.");  
  }
  
  if (WRITE_DEMO_FILE) {
    Serial.print("Writing demo file to card... ");
    File dataFile = SD.open("DEMOFILE.txt", FILE_WRITE);
    
    if (dataFile) {
      dataFile.println("Hello, world!");
      dataFile.close();
      Serial.println("Demo write successful.");
    }
    else {
      Serial.print("Demo write failed..");
    }
  }

  /* 2. Initialize the orientation sensor. */
  Serial.print("Initializing orientation sensor... ");
  if(!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true); // Synchronize the collection of data using the Feather's clock, not the one on-board the BNO.
  Serial.println("Sensor initialized.");

  /* 3. Write a header to the SD card file. */
  writeHeader(CSV_name, HEADER);
}


void loop() {
  /* 1. Collect BNO055 sensor data. */
  sensors_event_t event; // Associate memory with an event object
  bno.getEvent(&event);  // Send the orientation sensor's data to the in-memory address of the event object

  orientation    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration   = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  magnetic_field = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  temperature = bno.getTemp();
  
  /* 2. Collect force data. */
  fsrReading  = analogRead(fsrPin);
  force       = map(fsrReading, 0, 1023, 0, 100);
  
  /* 3. Write measurements to the SD card. */
  writeData(CSV_name,
            12,
            (int) time_step,
            (int) orientation.x(),
            (int) orientation.y(),
            (int) orientation.z(),
            (int) (ACCELERATION_SF * acceleration.x()),
            (int) (ACCELERATION_SF * acceleration.y()),
            (int) (ACCELERATION_SF * acceleration.z()),
            (int) magnetic_field.x(),
            (int) magnetic_field.y(),
            (int) magnetic_field.z(),
            (int) force,
            (int) temperature
            );

  time_step  += 1;
  delay(sampling_period);
}
      

void writeData(String filename, int n_args, ...){
  String valString;
  va_list argList;
  dataFile = SD.open(filename, FILE_WRITE);
  
  if (dataFile){
    va_start(argList, n_args);
    for (int i = 0; i < n_args; i++) { // Print each argument to a line in the file
      valString = String(va_arg(argList, int));
      dataFile.print(valString + ",");
    }
    va_end(argList);
    
    dataFile.println("");
    dataFile.close();
  }
  else{
    Serial.println("Error writing data to file.");
  }
}

void writeHeader(String filename, String header){
  dataFile = SD.open(filename, FILE_WRITE);
  
  if (dataFile) { dataFile.println(header); }
  else{ Serial.println("Error writing header to file."); }
  
  dataFile.close();
}
