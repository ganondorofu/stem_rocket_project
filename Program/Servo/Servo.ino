#include <Servo.h>

static Servo s_servo; /**< Servo object */

/**
 * @brief Initialize servo motor and serial communication.
 */
void setup() {
  /* Initialize serial communication */
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Servo Control Program Started");

  /* Attached to a servo motor whose input is connected to PIN_D06 
     Note: The pin selected must support PWM. */
  if (s_servo.attach(PIN_D06) == 0) {
    Serial.println("Error: Failed to attach servo to PIN_D06");
  } else {
    Serial.println("Servo successfully attached to PIN_D06");
  }

  /* Set the servo motor angle to 90 degrees */
  s_servo.write(90);
  delay(500); // Give the servo some time to move
  int currentPos = s_servo.read();
  Serial.print("Attempting to set servo to 90 degrees. Current position: ");
  Serial.println(currentPos);

  if (abs(currentPos - 90) > 5) {
    Serial.println("Warning: Servo position may not be accurate");
  }

  /* Wait 5000ms */
  Serial.println("Waiting for 5 seconds...");
  delay(5000);
}

/**
 * @brief Change servo motor angle.
 */
void loop() {
  moveAndCheckServo(0);
  delay(1000);

  moveAndCheckServo(180);
  delay(1000);
}

/**
 * @brief Move servo to a specified angle and check the result.
 * @param angle he angle to move the servo to.
 */
void moveAndCheckServo(int angle) {
  Serial.print("Attempting to move servo to ");
  Serial.print(angle);
  Serial.println(" degrees");

  s_servo.write(angle);
  delay(500); // Give the servo some time to move

  int currentPos = s_servo.read();
  Serial.print("Current servo position: ");
  Serial.println(currentPos);

  if (abs(currentPos - angle) > 5) {
    Serial.println("Warning: Servo position may not be accurate");
  }
}