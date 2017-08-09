#include <imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


/* > General Declarations < */
const bool PRINT_MEASUREMENTS = false;
const int  SAMPLING_PERIOD    = 10;    // ms
int        time_step          = 0;

/* > Datalogger Declarations < */
const String HEADER          = "time,th_x,th_y,th_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,grav_x,grav_y,grav_z,force,temperature,calibration";
const bool   WRITE_DEMO_FILE = false;
const int    CS_PIN          = 4;
int          file_index      = 0;
String       CSV_name;
File         dataFile;
File         root;

/* > Orientation Sensor Declarations < */
Adafruit_BNO055 bno                  = Adafruit_BNO055(55);   // What does the 55 do?
const int      CALIBRATION_THRESHOLD = 100;                   // 200 * 100 = 10000ms -> Continuous length of time the sensor must be calibrated for before data collection begins.
const int      ACCELERATION_SF       = 100;                   // Retains precision of acceleration floats when casting to int.
int            calibration_score     = 0;                     // Accumulator for calibration phase
int            *calibration          = NULL;                  // Pointer to an array of calibration status values.
imu::Vector<3> magnetic_field;
imu::Vector<3> acceleration;
imu::Vector<3> orientation;
imu::Vector<3> gravity;
int            temperature;

/* > Force Sensor Declarations < */
const int FSR_PIN         = 0;     // the FSR and 10K pulldown are connected to a0
int       force_reading;
int       force;


void setup(void) {
  Serial.begin(9600);
 
  /* 1. Initialize the SD card. */
  Serial.print("Initializing SD card... ");
  
  pinMode(CS_PIN, OUTPUT);
  if (!SD.begin(CS_PIN)) {
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
  dataFile.println(HEADER);


  /* 2. Initialize the orientation sensor. */
  Serial.print("Initializing orientation sensor... ");
  if(!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);                         // Synchronize the collection of data using the Feather's clock, not the one on-board the BNO.
  Serial.println("Sensor initialized.");

  /* 3. Calibrate the orientation sensor. */
  Serial.println("Calibrating... ");
  while(calibration_score < CALIBRATION_THRESHOLD) {  // Requires at least CALIBRATION_THRESHOLD successive (calibration == 3) events
    digitalWrite(13, LOW);                            // Feather's LED blinks during calibration
    delay(100);
    digitalWrite(13, HIGH);
    delay(100);
    calibration = getCalibrationStatus(bno);          // Returns a pointer to the first element of an int[4] array
    Serial.println("Calibration status:\t" + String(*(calibration))   + "\t" +
                                             String(*(calibration+1)) + "\t" +
                                             String(*(calibration+2)) + "\t" +
                                             String(*(calibration+3)));
    
    calibration_score = max(*(calibration) - 2, 0)*(calibration_score + *(calibration + 2) - 2); // Accumulator
  }
  Serial.println("Sensor calibrated.");

  delay(1000);
}


void loop() {
  /* 1. Collect BNO055 sensor data. */
  sensors_event_t event; // Allocate memory to an event object
  bno.getEvent(&event);  // Send the orientation sensor's data to the in-memory address of the event object

  magnetic_field = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  acceleration   = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gravity        = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  orientation    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  calibration    = getCalibrationStatus(bno);
  temperature    = bno.getTemp();
  
  /* 2. Collect force data. */
  force_reading  = analogRead(FSR_PIN);
  force          = map(force_reading, 0, 1023, 0, 100);
  
  /* 3. Write measurements to the SD card. */
  writeData(dataFile,
            16, // Number of further arguments
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

  /* 4. Periodically save the data. */
  if (time_step % 1000 == 0){ // Should really introduce a save button
    dataFile.close();
    dataFile = SD.open(CSV_name, FILE_WRITE);
    }

  /* 5. Print calibration status to monitor. */ 
  calibration = getCalibrationStatus(bno);
  Serial.println("Data collection status:\t" + String(*(calibration))   + "\t" +
                                               String(*(calibration+1)) + "\t" +
                                               String(*(calibration+2)) + "\t" +
                                               String(*(calibration+3)));
  time_step  += 1;
  delay(SAMPLING_PERIOD);
}
      

void writeData(File file, int n_args, ...){
  /* Writes a variable number of integer arguments to a file in CSV format. */
  va_list argList;
  
  va_start(argList, n_args);
  for (int i = 0; i < n_args; i++) { // Print each argument to a line in the file
    file.print(String(va_arg(argList, int)) + ",");
  }
  va_end(argList);
  
  file.println("");
}


void writeDemoFile(){
  /* Writes a text file to verify that the SD card can be written to. */
  Serial.print("Writing demo file to card... ");
  File dataFile = SD.open("DEMOFILE.txt", FILE_WRITE);
  
  if (dataFile) {
    dataFile.println("Hello, world!");
    dataFile.close();
    Serial.println("Demo write successful.");
  }
  else {
    Serial.print("Demo write failed.");
  }
}


int * getCalibrationStatus(Adafruit_BNO055 bno){
    /* Returns a pointer to an array of calibration statuses {sys, gyro, accel, mag}. */
    static int calibration_status[4]; // Only run once (regardless of no. of function calls)
    static uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    calibration_status[0] = (int) system;
    calibration_status[1] = (int) gyro;
    calibration_status[2] = (int) accel;
    calibration_status[3] = (int) mag;
    return calibration_status;
}
