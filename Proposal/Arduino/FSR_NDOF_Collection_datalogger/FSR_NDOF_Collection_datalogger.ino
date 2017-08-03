#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


/* > General Declarations < */
const int time_step = 50;      // samping period of sensors
const int datapts = 400;       // number of data points collected before stored in the datalogger
int arr_counter = 0;           // counts the datapoints and acts as ID (1,2,3...) in the .csv file
const bool PRINT_MEASUREMENTS = false;
const bool WRITE_DEMO_FILE    = true;

/* > Datalogger Declarations < */
const int CSpin = 4;
File dataFile;
String CSVName = "0803001.csv";
Adafruit_BNO055 bno = Adafruit_BNO055(55); // What does the 55 do?

/* > Orientation Sensor Declarations < */

/* > Force Sensor Declarations < */
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;
int fsrRemap;


void setup(void) {

  delay(5000);
  Serial.begin(9600);
 
  /* 1. Attempt to initialize the SD card. */
  Serial.print("Initializing SD card... ");
  pinMode(CSpin, OUTPUT);
 
  if (!SD.begin(CSpin)) {
    Serial.println("Card failed, or not present.");
    return;
  }
  
  Serial.println("Card initialized.");  
  delay(100); //wdt safety?
  
  if (WRITE_DEMO_FILE) {
    Serial.print("Writing demo file to card... ");
    File dataFile = SD.open("demofile.txt", FILE_WRITE);
    
    if (dataFile) {
      dataFile.println("Hello, world!");
      dataFile.close();
      Serial.println("Demo write successful.");
    }
    else {
      Serial.print("Demo write unsuccessful.");
    }
  }
  /* 2. Initialize the orientation sensor. */
  Serial.print("Initializing orientation sensor... ");
  
  if(!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Sensor initialized.");
  
  delay(1000); // Why?
    
  bno.setExtCrystalUse(true); // Synchronize the collection of data using the Feather's clock, not the one on-board the BNO.
}


void loop() {
   String euler_x = "0";
   String euler_y = "0";
   String euler_z = "0";
   String accel_x = "0";
   String accel_y = "0";
   String accel_z = "0";
   String arr_string = "0";
   String fsr_touch = "0";
   arr_counter = 1;

   /* 1. Collect <datapts> measurements and store them in strings. */
    while (arr_counter < datapts) {
      
      /* 1.1 Collect orientation and acceleration data. */
      sensors_event_t event; // Associate memory with an event object
      bno.getEvent(&event);  // Send the orientation sensor's data to the in-memory address of the event object
      
      euler_x += "," +  String(event.orientation.x); // azimuth, pitch, roll?;
      euler_y += "," +  String(event.orientation.y);
      euler_z += "," +  String(event.orientation.z);

      accel_x += "," + String(event.acceleration.x);
      accel_y += "," + String(event.acceleration.y);
      accel_z += "," + String(event.acceleration.z);

      // I think we can use event.orientation.v to retrieve all components simultaneously as a list
      
      /* 1.2. Collect force data. */
      fsrReading = analogRead(fsrPin);
      fsrRemap = map(fsrReading, 0, 1023, 0, 100);
      fsr_touch += "," + String(fsrRemap);
  
      /* 1.3. Print readings to the monitor. */
      if (PRINT_MEASUREMENTS) {
        Serial.print("Analog reading = ");
        Serial.print(fsrReading);
     
        if (fsrReading < 10) {
          Serial.println(" - No pressure");
        } else if (fsrReading < 200) {
          Serial.println(" - Light touch");
        } else if (fsrReading < 500) {
          Serial.println(" - Light squeeze");
        } else if (fsrReading < 800) {
          Serial.println(" - Medium squeeze");
        } else {
          Serial.println(" - Big squeeze");
        }
        
        Serial.print("X orientation: ");
        Serial.print(event.orientation.x);
        Serial.print("\tY orientation: ");
        Serial.print(event.orientation.y);
        Serial.print("\tZ orientation: ");
        Serial.print(event.orientation.x);
        Serial.println("");
      }
      
      /* 1.4. Increment the datapoint counter. */
      arr_counter += 1;
      arr_string  += "," + String(arr_counter);
      
      delay(time_step);
    }

  
  /* 2. Write the strings to the datalogger. */
  saveData(arr_string, CSVName);
  saveData(euler_x, CSVName);
  saveData(euler_y, CSVName);
  saveData(euler_z, CSVName);
  saveData(accel_x, CSVName);
  saveData(accel_y, CSVName);
  saveData(accel_z, CSVName);
  saveData(fsr_touch, CSVName);

  Serial.println("Step index: " + arr_string);
  Serial.println("Orientation w.r.t. x-axis:" + euler_x);
  
  delay(5000); // Why?      
  }
      

void saveData(String dataString, String filename){
  dataFile = SD.open(filename, FILE_WRITE);
  
  if (dataFile){
    dataFile.println(dataString);
    dataFile.close();
  }
  else{
    Serial.println("Error writing to file.");
  }
}
