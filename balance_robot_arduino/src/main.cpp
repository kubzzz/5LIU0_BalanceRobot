/*
 * Balance robot main arduino code
 * 
 * Authors: 
 * Martijn Strolenberg
 * Tom Janssen-Bouwmeester
 * 
 * I2C Device at address 0x1E, 0x68, 0x7E found
 *
 */

/* Libraries used */
#include <Arduino.h>
#include <ICM42688.h>     // ICM42688 IMU Library
#include <TMC2209.h>      // TMC2209 stepper drivers Library (wrote softwareserial to false)
#include <AccelStepper.h> // Accel Stepper Library 
#include <TMCStepper.h>

/* common defines */
#define LED_PIN 13 // Built in LED of Arduino

#define MOTOR_ENABLE_PIN 8
#define STEP_PIN_R 11
#define DIR_PIN_R 12
#define STEP_PIN_L 9
#define DIR_PIN_L 10

#define SERIAL_BAUD_RATE 115200
#define DRIVER_ADDRESS 0b10 // UART address of a TMC2209 driver defined by MS1 and MS2 
#define SERIAL_PORT Serial1 //Hardware serial used Serial1-->(TX=18, RX=19)
#define R_SENSE 0.11f // current sense resistor used (0.11ohm)

#define MICRO_STEPS 16

#define WIRE Wire1 // Wire->(SDA, SCL) Wire1-->(SDA1, SCL1)

void print_IMU_data(void); // Function to print IMU data

/* Making IMU object with I2C channel (Wire1) and address (0x68) */
ICM42688 IMU(WIRE, 0x68);

/* Making an motor object */
AccelStepper motor_right = AccelStepper(motor_right.DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper motor_left = AccelStepper(motor_left.DRIVER, STEP_PIN_L, DIR_PIN_L);

/* Enable the UART communication with TMC2209 driver */
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

int speed; // speed variable

void setup() {
  // put your setup code here, to run once:
  /* Set pinModes */
  pinMode(STEP_PIN_R, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  pinMode(STEP_PIN_L, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  digitalWrite(MOTOR_ENABLE_PIN, LOW); // set HIGH to disable drivers set LOW to enable
  Serial.begin(SERIAL_BAUD_RATE); // begin serial communication to read data from the arduino
  while (!Serial) {}

  /* Begin serial communication with TMC2209 drivers */
  SERIAL_PORT.begin(SERIAL_BAUD_RATE);
  driver.begin();
  //driver.toff(5);
  driver.rms_current(1000); // set current in mA
  driver.microsteps(MICRO_STEPS); // set micro step in 1/MICRO_STEPS steps
  driver.pwm_autoscale(true);

  /* Set Max Speed is in steps per second */ 
  motor_right.setMaxSpeed(5000); // 100mm/s @ 80 steps/mm
  motor_left.setMaxSpeed(5000);

  /* Set Acceleration in steps per second squared */
  motor_right.setAcceleration(200); // 2000mm/s^2
  motor_left.setAcceleration(200);
  //stepper.setEnablePin(EN_PIN);

  /* direction, step, enable inverted */
  motor_right.setPinsInverted(false, false, true);
  motor_left.setPinsInverted(true, false, true);

  motor_right.enableOutputs();
  motor_left.enableOutputs();


  int status = IMU.begin();
  /* Check if the IMU is correctly working */
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("Succesfull initialized IMU");

  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelFS(ICM42688::gpm8);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroFS(ICM42688::dps500);
  
  // set output data rate to 12.5 Hz
  IMU.setAccelODR(ICM42688::odr12_5);
  IMU.setGyroODR(ICM42688::odr12_5);

  Serial.println("ax,ay,az,gx,gy,gz,temp_C");


}
bool shaft = false;
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0)
  {
    speed = Serial.parseInt();
  
    // Print speed output
    Serial.print("Current Speed ");                 
    Serial.print(speed);
    Serial.println(" steps per second.");
   
  }
  motor_right.setSpeed(speed);
  motor_left.setSpeed(speed);
  
  motor_right.run();
  motor_left.run();
  // print_IMU_data();

  
}


void print_IMU_data(void)
{
  // read the sensor
  IMU.getAGT();
  // display the data
  Serial.print(IMU.accX(),6);
  Serial.print("\t");
  Serial.print(IMU.accY(),6);
  Serial.print("\t");
  Serial.print(IMU.accZ(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrX(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrY(),6);
  Serial.print("\t");
  Serial.print(IMU.gyrZ(),6);
  Serial.print("\t");
  Serial.println(IMU.temp(),6);
  delay(20);
}
