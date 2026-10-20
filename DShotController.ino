/*
Charles Waldron and Samuel Smith

This code can be found on Github (Teams)
  Athena > !!Avionics > !Github
Please make sure you have the must up to date version before working on it
*/


#include <Arduino.h> 
//#include <DShotRMT.h>// ESC includes
#include <SPI.h>// IMU includes
#include "SparkFun_BMI270_Arduino_Library.h" 
#include "GNC_Types.h" 

// Change motor pin to actual
// This will be the ESC pin
const gpio_num_t MOTOR_PIN = GPIO_NUM_14; //GPIO_NUM_13, GPIO_NUM_15

const float alpha = .08;
constexpr float DPS_TO_RAD = 0.01745329252f; // Custom define of DEG_TO_RAD so it stays as a float
//const float dt = 0.000625;
Vec3D rawAlpha;
Vec3D filteredAlpha;
Vec3D omega, lomega;
Vec3D r_COR = {.3, .2, .8}; // Radius of the IMU from the Center of Rotation
Vec3D g_B;
Vec3D linAlpha;
int firstReading = 1;

// IMU Defines
#define IMU_INT1_PIN 39 // INT1 
#define SPI_CS   38     // CS
#define SPI_MOSI 35     // SDA
#define SPI_SCK  36     // SCL
#define SPI_MISO 37     // ADR

BMI270 imu;
uint32_t spiClockSpeed = 2000000; //2 MHz 
volatile bool dataReadyFlag = false;

// Create a sensor data object
//sensorData sD;

// Create an instanice of the DShot Motor
//DShotRMT motor(MOTOR_PIN, DSHOT300, false);

// Performace trackers, these can be removed after testing
uint32_t lastReportTime = 0;

//LEAVE THIS FUNCTION ALONE
//An interrupt must not have anything else inside it
void IRAM_ATTR imuISR() {
  dataReadyFlag = true; 
}

void setup() {
  // Set the Serial rate
  Serial.begin(115200);

  // Wait for serial to become initalized
  while(!Serial) {
    delay(10); 
  }

  pinMode(IMU_INT1_PIN, INPUT); // Attach the interrupt

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

  // Write the flash file to to the IMU
  while (imu.beginSPI(SPI_CS, spiClockSpeed, SPI) != BMI2_OK) { //
    Serial.println("Error: BMI270 not detected via SPI. Check wiring and CS pin!");
    delay(1000); 
  }

  // Set up the IMU and attach its interrupt pin

  imu.setAccelODR(BMI2_ACC_ODR_1600HZ);
  imu.setGyroODR(BMI2_GYR_ODR_1600HZ);
  imu.setAccelPowerMode(1); // Setting a one here should set the IMU into high performance mode
  imu.setGyroPowerMode(1, 1); // Sets it with filter mode and noise mode both as performance mode

  // Set the interrupt to INT1
  bmi2_int_pin_config intPinConfig{};
  intPinConfig.pin_type = BMI2_INT1;                  // Directing to INT1 Pin
  intPinConfig.int_latch = BMI2_INT_NON_LATCH;        // Clear latches natively

  // Pin properties are array-indexed underneath for physical pin registers [0]
  intPinConfig.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_HIGH;   // Active High flag
  intPinConfig.pin_cfg[0].od        = BMI2_INT_PUSH_PULL;     // Push-Pull engine
  intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; // Enable output path
  intPinConfig.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;  // Disable input path

  imu.setInterruptPinConfig(intPinConfig);
  imu.mapInterruptToPin(BMI2_DRDY_INT, BMI2_INT1);

  // and attach it to the ESP
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);

  // Initalize the motor
  //motor.begin(); 
  Serial.println("System initialized finished...");
}


void loop() {
  // Tuning instructions:
  // If the motor is acting too violently, decrease alpha
  // If the motor is acting too sluggishly, increase alpha

  // Update the motor as needed
  //motor.sendThrottlePercent(50);

  if (dataReadyFlag) {
    dataReadyFlag = false; // Reset flag immediately

    // Get the data from the SPI connection. Should take ~60-70 microseconds 
    uint32_t startUs = micros();
    int8_t result = imu.getSensorData();
    uint32_t readUs = micros() - startUs;

    // If the read failed, toss it out
    if (result != BMI2_OK) {
      // Skip processing for the failed read
      return;
    }

    // ---- DO NOT PUT SERIAL PRINTS OR DELAYS HERE ----
    // 1. Poll sensor data
    // 2. Apply the Low Pass Filter
    // 3. Push omega to lomega
    // 4. Compute the numerical derivative
    // 5. Compute the new gravity in the body frame 

    // Filter the alphas
    if (firstReading) {
      firstReading = 0;
      rawAlpha = {0.0f, 0.0f, 0.0f};
      filteredAlpha = {0.0f, 0.0f, 0.0f};
    } else {
      omega.x = imu.data.gyroX * DPS_TO_RAD;
      omega.y = imu.data.gyroY * DPS_TO_RAD;
      omega.z = imu.data.gyroZ * DPS_TO_RAD;
      
      // Compute the numerical derivative
      rawAlpha.x = (omega.x - lomega.x) / readUs;
      rawAlpha.y = (omega.y - lomega.y) / readUs;
      rawAlpha.z = (omega.z - lomega.z) / readUs;

      // Apply a LPF
      filteredAlpha.x = (alpha * rawAlpha.x) + ((1.0 - alpha) * filteredAlpha.x);
      filteredAlpha.y = (alpha * rawAlpha.y) + ((1.0 - alpha) * filteredAlpha.y);
      filteredAlpha.z = (alpha * rawAlpha.z) + ((1.0 - alpha) * filteredAlpha.z);
    }

    // Push the current omegas down to lomega
    lomega = {omega.x, omega.y, omega.z};

    // Compute the gravity in the body frame
    // Equation from the GNC Document
    linAlpha = {imu.data.accelX, imu.data.accelY, imu.data.accelZ}; // Convert the linear alphas to a Vec3D for calculations
    g_B = linAlpha - filteredAlpha.cross(r_COR) - lomega.cross(lomega.cross(r_COR));
  }

  // Some operation to perfrom only once per set time interval
  // This is set to 100 ms
  if (millis() - lastReportTime >= 100) {
    lastReportTime = millis();

    Serial.printf(
  "Gyro rad/s: %.3f %.3f %.3f | Accel g: %.3f %.3f %.3f\n",
  lomega.x, lomega.y, lomega.z,
  linAlpha.x, linAlpha.y, linAlpha.z);
  }
}
