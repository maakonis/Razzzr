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
int time_step             = 0;

/* > Datalogger Declarations < */
const String HEADER = "time,th_x,th_y,th_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,grav_x,grav_y,grav_z,force,temperature,calibration";
String CSV_name;
const int CSpin = 4;
File dataFile;
File root;
int file_index = 0;

/* > Orientation Sensor Declarations < */
Adafruit_BNO055 bno = Adafruit_BNO055(55); // What does the 55 do?
imu::Vector<3> orientation;
imu::Vector<3> acceleration;
imu::Vector<3> magnetic_field;
imu::Vector<3> gravity;
const int ACCELERATION_SF = 100;
int temperature;
int *calibration = NULL;

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
  else { Serial.println("Card initialized.");  }
  
  if (WRITE_DEMO_FILE) { writeDemoFile(); }

  root = SD.open("/");
  while(root.openNextFile()) {
    file_index += 1;
  }
  CSV_name = String(file_index) + ".CSV";
  
  dataFile = SD.open(CSV_name, FILE_WRITE);

  /* 2. Initialize the orientation sensor. */
  Serial.print("Initializing orientation sensor... ");
  if(!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true); // Synchronize the collection of data using the Feather's clock, not the one on-board the BNO.
  Serial.println("Sensor initialized.");

  Serial.println("Calibrating... ");
  while(calibration[0] != 3) {
    // Feather's LED blinks during calibration
    digitalWrite(13, LOW);
    delay(100);
    digitalWrite(13, HIGH);
    delay(100);
    calibration = getCalibrationStatus(bno); // Returns a pointer to the first element of an int[4] array
    Serial.println("Calibration status: " + String(*(calibration)) + "\t" +
                                            String(*(calibration+1)) + "\t" +
                                            String(*(calibration+2)) + "\t" +
                                            String(*(calibration+3)));
  }
  Serial.println("Sensor calibrated.");

  /* 3. Write a header to the SD card file. */
  dataFile.println(HEADER);
}


void loop() {
  /* 1. Collect BNO055 sensor data. */
  sensors_event_t event; // Associate memory with an event object
  bno.getEvent(&event);  // Send the orientation sensor's data to the in-memory address of the event object

  orientation    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration   = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  magnetic_field = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  gravity        = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  temperature    = bno.getTemp();
  calibration    = getCalibrationStatus(bno);
  
  /* 2. Collect force data. */
  fsrReading  = analogRead(fsrPin);
  force       = map(fsrReading, 0, 1023, 0, 100);
  
  /* 3. Write measurements to the SD card. */
  writeData(dataFile,
            16,
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
            (int) (ACCELERATION_SF * gravity.x()),
            (int) (ACCELERATION_SF * gravity.y()),
            (int) (ACCELERATION_SF * gravity.z()),
            (int) force,
            (int) temperature,
            (int) *calibration
            );

  time_step  += 1;

  if (time_step % 100 == 0){
    // Save data periodically
    dataFile.close();
    dataFile = SD.open(CSV_name, FILE_WRITE);
    }

   calibration = getCalibrationStatus(bno);
   Serial.println("Data collection status: " + String(*(calibration)) + "\t" +
                                               String(*(calibration+1)) + "\t" +
                                               String(*(calibration+2)) + "\t" +
                                               String(*(calibration+3)));
}
      

void writeData(File file, int n_args, ...){
  String valString;
  va_list argList;
  
  va_start(argList, n_args);
  for (int i = 0; i < n_args; i++) { // Print each argument to a line in the file
    valString = String(va_arg(argList, int));
    file.print(valString + ",");
  }
  va_end(argList);
  
  file.println("");
}


void writeDemoFile(){
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

int * getCalibrationStatus(Adafruit_BNO055 bno){
    static int calibration_status[4]; // Only run once (regardless of no. of function calls)
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    calibration_status[0] = (int) system;
    calibration_status[1] = (int) gyro;
    calibration_status[2] = (int) accel;
    calibration_status[3] = (int) mag;
    return calibration_status; // Pointer to first element of calibration_status!
}
