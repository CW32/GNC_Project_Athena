/**
 * @file DShotController.ino
 * @brief Main body for GNC Project Athena Tech demonstrator
 *
 * @author Charles Waldron
 * @date 2026-11-09
 */

#include <Arduino.h> 
//#include <DShotRMT.h> 
#include <SPI.h>
#include "../BMI270/src/SparkFun_BMI270_Arduino_Library.h" 
#include "Vec3D.h" 
#include "PID.h"
#include "driver/gpio.h"
#include "LPF.h"
#include "NumericalDerivative.h"

// Change motor pin to actual
// This will be the ESC pin
const gpio_num_t MOTOR_PIN = GPIO_NUM_14; //GPIO_NUM_13, GPIO_NUM_15
const gpio_num_t STATE = GPIO_NUM_8; // [A5]

const float alpha = .08;
constexpr float DPS_TO_RAD = 0.01745329252f; // Custom define of DEG_TO_RAD so it stays as a float
constexpr float GRAVITY = 9.80665f;
//const float dt = 0.000625;
GNC::Math::Vec3D rawAlpha;
GNC::Math::Vec3D filteredAlpha;
GNC::Math::Vec3D omega, lomega;
GNC::Math::Vec3D r_COR = {.3, .2, .8}; // Radius of the IMU from the Center of Rotation
GNC::Math::Vec3D g_B;
GNC::Math::Vec3D linAlpha;
int firstReading = 1;
float resultX, resultY, resultZ;

// IMU Defines
#define IMU_INT1_PIN 39 // INT1 [TX]
#define SPI_CS   38     // SCK  [RX] // enable
#define SPI_MOSI 35     // SDA  
#define SPI_SCK  36     // SCL 
#define SPI_MISO 37     // ADR

BMI270 imu;
uint32_t spiClockSpeed = 2000000; //2 MHz 
volatile bool dataReadyFlag = false;

// Create an instanice of the DShot Motor
//DShotRMT motor(MOTOR_PIN, DSHOT300, false);

// Performace trackers, these can be removed after testing
uint32_t lastReportTime = 0;

//LEAVE THIS FUNCTION ALONE
//An interrupt must not have anything else inside it
void IRAM_ATTR imuISR() {
  dataReadyFlag = true; 
}

GNC::Tools::PID controllerX{0.5f, 0.0f, 0.0f, -1.0f, 1.0f, -10.0f, 10.0f};
GNC::Tools::PID controllerY{0.5f, 0.0f, 0.0f, -1.0f, 1.0f, -10.0f, 10.0f};
GNC::Tools::PID controllerZ{0.5f, 0.0f, 0.0f, -1.0f, 1.0f, -10.0f, 10.0f};

GNC::Tools::LPF lpfX{.15, 0};
GNC::Tools::LPF lpfY{.15, 0};
GNC::Tools::LPF lpfZ{.15, 0};

bool state = false;

/**
 * @brief Toggle a state pin to indicate the current execution step
 * 
 * @return void
 */
void toggle() {
  state = !state;
  gpio_set_level(STATE, state);
}


/**
 * @brief Set up code for Serial, SPI, BMI270, DShot controller
 * 
 * @return void
 */
void setup() {
  // Set the Serial rate
  Serial.begin(115200);

  // Wait for serial to become initalized
  while(!Serial) {
    delay(10); 
  }

  pinMode(IMU_INT1_PIN, INPUT); // Attach the interrupt

  // Set up State pin and set it low
  pinMode(STATE, OUTPUT); 
  gpio_set_level(STATE, 0);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

  // Write the flash file to to the IMU
  while (imu.beginSPI(SPI_CS, spiClockSpeed, SPI) != BMI2_OK) {
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

  // Disable the 450 us delay and change it to a 2 us delay
  // reduces L Sensor Data stage in processing
  if (!(imu.disableAdvancedPowerSave())) {
    Serial.println("Disable Power Save failed\n");
  }

  // and attach it to the ESP
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuISR, RISING);

  // Initalize the motor
  //motor.begin(); 
  Serial.println("System initialized finished...");
}


/**
 * @brief Code to run repeatedly 
 * 
 * @return void
 */
void loop() {
  // Tuning instructions:
  // If the motor is acting too violently, decrease alpha
  // If the motor is acting too sluggishly, increase alpha

  // Update the motor as needed
  //motor.sendThrottlePercent(50);


  if (dataReadyFlag) {
    dataReadyFlag = false; // Reset flag immediately

    // Get the data from the SPI connection. Should take ~60-70 microseconds 
    toggle(); //HIGH
    uint32_t startUs = micros();
    toggle();//Low
    int8_t result = imu.getSensorData();
    toggle();//High
    uint32_t readUs = micros() - startUs;

    // Convert the uint32_t to a float seconds
    float readToSec = static_cast<float>(readUs) * 1.0e-6f;

    // If the read failed, toss it out
    if (result != BMI2_OK) {
      // Skip processing for the failed read
      return;
    }

    toggle(); //LOW

    // Compute the filtered alpha values for linear acceleration
    omega.x = imu.data.gyroX * DPS_TO_RAD;
    omega.y = imu.data.gyroY * DPS_TO_RAD;
    omega.z = imu.data.gyroZ * DPS_TO_RAD;

    rawAlpha.x = GNC::Tools::numericalDerivative(omega.x, lomega.x, readToSec);
    rawAlpha.y = GNC::Tools::numericalDerivative(omega.y, lomega.y, readToSec);
    rawAlpha.z = GNC::Tools::numericalDerivative(omega.z, lomega.z, readToSec);

    filteredAlpha.x = lpfX.updateFilter(rawAlpha.x);
    filteredAlpha.y = lpfX.updateFilter(rawAlpha.y);
    filteredAlpha.z = lpfX.updateFilter(rawAlpha.z);

    toggle(); //HIGH

    // Push the current omegas down to lomega
    lomega = {omega.x, omega.y, omega.z};

    // Compute the gravity in the body frame
    // Equation from the GNC Document
    linAlpha = {imu.data.accelX, imu.data.accelY, imu.data.accelZ}; // Convert the linear alphas to a Vec3D for calculations
    g_B = linAlpha - filteredAlpha.cross(r_COR) - lomega.cross(lomega.cross(r_COR));

    toggle(); //LOW

    // Use the PID to control the inputs back down to zero
    resultX = controllerX.update(0.0f, g_B.x, readToSec);
    resultY = controllerY.update(0.0f, g_B.y, readToSec);
    resultZ = controllerZ.update(9.81f, g_B.z, readToSec);

    // Useless lines to prevent unused variable warnings
    GNC::Math::Vec3D resV = {resultX, resultY, resultZ};
    GNC::Math::Vec3D doNothing = {0.0f, 0.0f, 0.0f};
    resV + doNothing;
  }

  // Some operation to perfrom only once per set time interval
  // This is set to 100 ms
  if (millis() - lastReportTime >= 100) {
    lastReportTime = millis();

    Serial.printf("Omega %8.3f %8.3f %8.3f,Alpha %11.6f %11.6f %11.6f,Accel %8.3f %8.3f %8.3f,PID %8.3f %8.3f %8.3f\n",
    lomega.x, lomega.y, lomega.z,
    filteredAlpha.x * GRAVITY, filteredAlpha.y * GRAVITY, filteredAlpha.z * GRAVITY,
    linAlpha.x, linAlpha.y, linAlpha.z,
    resultX, resultY, resultZ
    );
  }
}