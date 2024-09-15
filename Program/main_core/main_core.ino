#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <GNSS.h>
#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <Camera.h>

SDClass SD;  /**< SDClass object */ 

File myFile; /**< File object */ 

#define SUBCORE 1
#define CSV_FILENAME "sensor_data.csv"

SpNavData navData;
SpGnss gnss;
int imageCounter = 0;
unsigned long lastRecordTime = 0;
const unsigned long recordInterval = 100; // Record every 0.1 seconds

struct sensorDataMsg {
  float temperature;
  float humidity;
  float pressure;
  float altitude;
  float latitude;
  float longitude;
  float gpsAltitude;
  int fix;
  int satellites;
  char imageFilename[32];
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting setup...");

  initCamera();
  initGNSS();

  // Initialize SubCore
  int ret = MP.begin(SUBCORE);
  if (ret < 0) {
    Serial.println("Error: Failed to start SubCore");
    while (1);
  }

  // Initialize SD card
  if (!SD.begin()) {
    Serial.println("Error: SD card initialization failed. Please check the SD card.");
    while (1);
  }
  Serial.println("SD card initialized");

  // Initialize CSV file
  initCSV();

  Serial.println("Setup complete");
}

void loop() {
  // Capture image from camera
  captureImage();

  // Update GNSS data
  gnss.waitUpdate(-1);

  // Request and receive sensor data (every 0.1 seconds)
  if (millis() - lastRecordTime >= recordInterval) {
    requestAndReceiveSensorData();
    lastRecordTime = millis();
  }
}

void initCamera() {
  Serial.println("Initializing camera...");
  CamErr err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
    errorLoop(1);
  }

  Serial.println("Setting picture format...");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QUADVGA_H,
    CAM_IMGSIZE_QUADVGA_V,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
    errorLoop(1);
  }

  Serial.println("Camera initialized");
}

void initGNSS() {
  int result;

  result = gnss.begin();
  if (result != 0) {
    Serial.println("GNSS start error");
    errorLoop(2);
  }

  result = gnss.start();
  if (result != 0) {
    Serial.println("GNSS initialization error");
    errorLoop(2);
  }

  Serial.println("GNSS initialized");
}

void captureImage() {
  char filename[32];
  snprintf(filename, sizeof(filename), "IMG_%04d.jpg", imageCounter);

  CamImage img = theCamera.takePicture();
  if (img.isAvailable()) {
    File myFile = SD.open(filename, FILE_WRITE);
    if (!myFile) {
      Serial.println("Failed to write file");
      return;
    }
    myFile.write(img.getImgBuff(), img.getImgSize());
    myFile.close();
    Serial.println("Image saved: " + String(filename));
    imageCounter++;
  } else {
    Serial.println("Failed to capture image");
  }
}

void requestAndReceiveSensorData() {
  int ret;
  int8_t msgid = 100;
  uint32_t dummy_data = 0;
  struct sensorDataMsg rcvMsg;

  // Send request to SubCore
  ret = MP.Send(msgid, dummy_data, SUBCORE);
  if (ret < 0) {
    Serial.println("Error: Failed to send message to SubCore");
    return;
  }

  // Receive data from SubCore
  MP.RecvTimeout(1000);
  ret = MP.Recv(&msgid, &rcvMsg, SUBCORE);
  if (ret < 0) {
    Serial.println("Error: Failed to receive message from SubCore");
    return;
  } else {
    // Print received sensor data
    Serial.println("Main Core: Sensor data received successfully");
    Serial.print("Temperature: "); Serial.print(rcvMsg.temperature); Serial.print(" C, ");
    Serial.print("Humidity: "); Serial.print(rcvMsg.humidity); Serial.print(" %, ");
    Serial.print("Pressure: "); Serial.print(rcvMsg.pressure); Serial.print(" hPa, ");
    Serial.print("Estimated Altitude: "); Serial.print(rcvMsg.altitude); Serial.print(" m, ");
    Serial.print("Acceleration X: "); Serial.print(rcvMsg.accelX); Serial.print(" g, ");
    Serial.print("Acceleration Y: "); Serial.print(rcvMsg.accelY); Serial.print(" g, ");
    Serial.print("Acceleration Z: "); Serial.print(rcvMsg.accelZ); Serial.print(" g, ");
    Serial.print("Gyro X: "); Serial.print(rcvMsg.gyroX); Serial.print(" deg/s, ");
    Serial.print("Gyro Y: "); Serial.print(rcvMsg.gyroY); Serial.print(" deg/s, ");
    Serial.print("Gyro Z: "); Serial.println(rcvMsg.gyroZ); Serial.print(" deg/s");
  }

  // Update GNSS data
  gnss.getNavData(&navData);
  rcvMsg.latitude = navData.latitude;
  rcvMsg.longitude = navData.longitude;
  rcvMsg.gpsAltitude = navData.altitude;
  rcvMsg.fix = navData.posFixMode;
  rcvMsg.satellites = navData.numSatellites;

  // Write data to CSV
  writeToCSV(rcvMsg);
}

void initCSV() {
  File dataFile = SD.open(CSV_FILENAME, FILE_WRITE);
  if (dataFile) {
    if (dataFile.size() == 0) {
      dataFile.println("Time(s),Temp(C),Humidity(%),Pressure(hPa),EstAlt(m),Lat,Lng,Alt(m),Fix,Sats,ImageFilename,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    }
    dataFile.close();
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

void writeToCSV(const sensorDataMsg& data) {
  Serial.println("Saving to CSV");
  File dataFile = SD.open(CSV_FILENAME, FILE_WRITE);
  if (dataFile) {
    char buffer[512];
    unsigned long currentTime = millis();
    float elapsedSeconds = currentTime / 1000.0;
    
    // Create formatted string
    int len = snprintf(buffer, sizeof(buffer), "%.3f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
             elapsedSeconds, data.temperature, data.humidity, data.pressure, data.altitude,
             data.latitude, data.longitude, data.gpsAltitude, data.fix, data.satellites, data.imageFilename,
             data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY, data.gyroZ);

    // Write formatted string to CSV file
    if (len >= 0 && len < sizeof(buffer)) {
      dataFile.println(buffer);
      Serial.println("CSV Line: " + String(buffer));
    } else {
      Serial.println("Error: Failed to create string with snprintf");
    }
    dataFile.close();
  } else {
    Serial.println("Error: Could not open CSV file");
  }
}

void printError(enum CamErr err) {
  Serial.print("Camera error: ");
  switch (err) {
    case CAM_ERR_NO_DEVICE: Serial.println("No device"); break;
    case CAM_ERR_ILLEGAL_DEVERR: Serial.println("Illegal device error"); break;
    case CAM_ERR_ALREADY_INITIALIZED: Serial.println("Already initialized"); break;
    case CAM_ERR_NOT_INITIALIZED: Serial.println("Not initialized"); break;
    case CAM_ERR_NOT_STILL_INITIALIZED: Serial.println("Still image not initialized"); break;
    case CAM_ERR_CANT_CREATE_THREAD: Serial.println("Failed to create thread"); break;
    case CAM_ERR_INVALID_PARAM: Serial.println("Invalid parameter"); break;
    case CAM_ERR_NO_MEMORY: Serial.println("Out of memory"); break;
    case CAM_ERR_USR_INUSED: Serial.println("Buffer already in use"); break;
    case CAM_ERR_NOT_PERMITTED: Serial.println("Operation not permitted"); break;
    default: Serial.println("Unknown error"); break;
  }
}

void errorLoop(int num) {
  while (1) {
    for (int i = 0; i < num; i++) {
      digitalWrite(LED3, HIGH);
      delay(300);
      digitalWrite(LED3, LOW);
      delay(300);
    }
    delay(1000);
  }
}