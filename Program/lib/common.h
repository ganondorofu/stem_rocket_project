#ifndef STEM_ROCKET_COMMON_H
#define STEM_ROCKET_COMMON_H

#include <Arduino.h>
#include <SDHCI.h>
#include <File.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <GNSS.h>
#include <Flash.h>

void initFlash(bool formatOnBoot, bool &flashInitialized);

bool initGNSS(SpGnssAddon &Gnss);

bool initBME280(Adafruit_BME280 &bme);

bool initMPU6050(MPU6050 &mpu);

bool initSDandCSV(SDClass &SD, String &csvFilename, String &logFilename,
                  bool &sdErrorHappened,
                  const char *CSV_BASE, const char *CSV_EXT,
                  const char *LOG_BASE, const char *LOG_EXT,
                  void (*handleError)(int));

bool initPreFlightCSV(SDClass &SD, String &preFlightFilename,
                      const char *PRE_FLIGHT_BASE, const char *PRE_FLIGHT_EXT,
                      void (*handleError)(int));

void logEvent(SDClass &SD, const String &logFilename, bool &sdErrorHappened,
              unsigned long startTime, const String &msg,
              void (*handleError)(int));

#endif
