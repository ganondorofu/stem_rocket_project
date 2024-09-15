#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MPU6050 mpu;

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
  Wire.begin();
  
  int ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }

  if (!bme.begin(0x76)) {
    errorLoop(3);
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    errorLoop(4);
  }

  Serial.println("SubCore: Setup complete");
}

void loop() {
  int ret;
  int8_t msgid;
  uint32_t dummy_data;

  ret = MP.Recv(&msgid, &dummy_data);
  if (ret < 0) {
    errorLoop(5);
  }

  struct sensorDataMsg msg;
  msg.temperature = bme.readTemperature();
  msg.humidity = bme.readHumidity();
  msg.pressure = bme.readPressure() / 100.0F;
  msg.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // Reading MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Converting raw values to meaningful units
  msg.accelX = ax / 16384.0;  // MPU6050 acceleration scale factor
  msg.accelY = ay / 16384.0;
  msg.accelZ = az / 16384.0;
  msg.gyroX = gx / 131.0;     // MPU6050 gyro scale factor
  msg.gyroY = gy / 131.0;
  msg.gyroZ = gz / 131.0;

  // GNSS data is set by the main core
  msg.latitude = 0;
  msg.longitude = 0;
  msg.gpsAltitude = 0;
  msg.fix = 0;
  msg.satellites = 0;
  strcpy(msg.imageFilename, "");  // Set by the main core

  ret = MP.Send(msgid, &msg);
  if (ret < 0) {
    Serial.println("SubCore: Send error");
    errorLoop(6);
  } else {
    Serial.println("SubCore: Send successful");
    // Print the sent data in one line upon successful transmission
    Serial.print("Sent data: Temperature: ");
    Serial.print(msg.temperature);
    Serial.print(" C, Humidity: ");
    Serial.print(msg.humidity);
    Serial.print(" %, Pressure: ");
    Serial.print(msg.pressure);
    Serial.print(" hPa, Estimated Altitude: ");
    Serial.print(msg.altitude);
    Serial.print(" m, Acceleration X: ");
    Serial.print(msg.accelX);
    Serial.print(" g, Acceleration Y: ");
    Serial.print(msg.accelY);
    Serial.print(" g, Acceleration Z: ");
    Serial.print(msg.accelZ);
    Serial.print(" g, Gyro X: ");
    Serial.print(msg.gyroX);
    Serial.print(" deg/s, Gyro Y: ");
    Serial.print(msg.gyroY);
    Serial.print(" deg/s, Gyro Z: ");
    Serial.print(msg.gyroZ);
    Serial.println(" deg/s");
  }
}

void errorLoop(int num) {
  while (1) {
    for (int i = 0; i < num; i++) {
      ledOn(LED0);
      delay(300);
      ledOff(LED0);
      delay(300);
    }
    delay(1000);
  }
}