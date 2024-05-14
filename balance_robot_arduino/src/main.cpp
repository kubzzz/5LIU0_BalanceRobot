/*
 * Balance robot main arduino code
 * 
 * Authors: 
 * Martijn Strolenberg
 * 
 * I2C Device at address 0x1E, 0x68, 0x7E found
 *
 */

/* Libraries used */
#include <Arduino.h>      // Standard Arduino functions
#include <ICM42688.h>     // ICM42688 IMU Library
#include <TMC2209.h>      // TMC2209 stepper drivers Library (wrote softwareserial to false)
#include <AccelStepper.h> // Accel Stepper Library 
#include <TMCStepper.h>   // 

/* common defines */
#define LED_PIN 13 // Built in LED of Arduino


/* Stepper driver defines */
#define MOTOR_ENABLE_PIN 8 // Pin to enable the motors
#define STEP_PIN_R 11 // Step pin of right motor
#define DIR_PIN_R 12 // Direction pin of right motor
#define STEP_PIN_L 9 // Step pin of left motor
#define DIR_PIN_L 10 // Direction pin of left motor
#define SERIAL_BAUD_RATE 115200 // serial baudrate for UART communication 
#define DRIVER_ADDRESS 0b10 // UART address of a TMC2209 driver defined by MS1 and MS2 
#define SERIAL_PORT Serial1 //Hardware serial used Serial1-->(TX=18, RX=19)
#define R_SENSE 0.11f // Current sense resistor used (0.11ohm)
#define MICRO_STEPS 16 // Amount of microsteps taken inside one full step (for more smooth motion)

/* IMU sensor defines */
#define WIRE Wire1 // Wire->(SDA, SCL) Wire1-->(SDA1, SCL1) for IMU sensor

/* Function Declaration */
void print_IMU_data(void); // Function to print IMU data
float calcAngle(float aY, float aZ, float gX); // Function to calculate angle(theta) arround the x-axis

HardwareSerial& serial_stream = Serial1; 
TMC2209 TMC2209_L;
TMC2209 TMC2209_R;

/* Making IMU object with I2C channel (Wire1) and address (0x68) */
ICM42688 IMU(WIRE, 0x68);

/* Making an motor object */
AccelStepper motor_right = AccelStepper(motor_right.DRIVER, STEP_PIN_R, DIR_PIN_R); // I don't want to use this
AccelStepper motor_left = AccelStepper(motor_left.DRIVER, STEP_PIN_L, DIR_PIN_L); // I don't want to use this

/* Enable the UART communication with TMC2209 driver */
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS); // I want to use this

int speed; // speed variable
bool shaft = false; // what does this do?

int prev_time = 0; // stores previous time in milliseconds
float theta_prev = 0.0; // stores previous angle theta in degrees 
float theta = 0.0; // stores current angle theta in degrees
float T_MS = 50.0; // Period of the control loop in milliseconds 

void setup() {
  // put your setup code here, to run once:
  /* Set pinModes */
  pinMode(STEP_PIN_R, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  pinMode(STEP_PIN_L, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  TMC2209_L.setup(serial_stream, 115200, TMC2209::SERIAL_ADDRESS_2); // ?
  TMC2209_R.setup(serial_stream, 115200, TMC2209::SERIAL_ADDRESS_1); // ?

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
  motor_right.setMaxSpeed(10000); // 100mm/s @ 80 steps/mm
  motor_left.setMaxSpeed(10000);

  /* Set Acceleration in steps per second squared */
  motor_right.setAcceleration(200); // 2000mm/s^2
  motor_left.setAcceleration(200);
  //stepper.setEnablePin(EN_PIN);

  /* direction, step, enable inverted */
  motor_right.setPinsInverted(false, false, true);
  motor_left.setPinsInverted(true, false, true);

  motor_right.enableOutputs();
  motor_left.enableOutputs();


  int status = IMU.begin(); // check status of IMU

  /* Check if the IMU is correctly working */
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("Succesfull initialized IMU");

  // setting the accelerometer full scale range to +/-2G
  IMU.setAccelFS(ICM42688::gpm2);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroFS(ICM42688::dps500);
  
  // set output data rate to 200 Hz
  IMU.setAccelODR(ICM42688::odr200);
  IMU.setGyroODR(ICM42688::odr200);


}

void loop() {
  if((millis() - prev_time) > T_MS) 
  {
    prev_time = millis();
  IMU.getAGT();
  // put your main code here, to run repeatedly:
    // if (Serial.available() > 0)
    // {
    //   speed = Serial.parseInt();
    //   // speed = (int);
    
    //   // Print speed output
    //   Serial.print("Current Speed ");                 
    //   Serial.print(speed);
    //   Serial.println(" steps per second.");
    
    // }
    motor_right.setSpeed(speed);
    motor_left.setSpeed(speed);
    
    motor_right.run();
    motor_left.run();
    // print_IMU_data();
    // Serial.println(calcAngle(IMU.accY(),IMU.accZ(), IMU.gyrX(), theta_prev));
    
    calcAngle(IMU.accY(),IMU.accZ(), IMU.gyrX());
    
  }
}

float calcAngle(float aY, float aZ, float gX) 
{
  Serial.print(">theta_prev: ");
  Serial.println(theta_prev);
  float phi = -(atan2(aZ,aY) * 57.f);
  theta = (0.95 * (theta_prev + (gX * (T_MS / 1000.0)))) + (0.05 * phi);
  
  // Serial.print("phi: ");
  // Serial.println(phi,6);
  Serial.print(">gX: ");
  Serial.println(gX);
  Serial.print(">gXdeltaT: ");
  Serial.println(gX * (T_MS / 1000.0),6);
  Serial.print(">angle: ");
  Serial.println(theta,6);
  theta_prev = theta;

  return theta;
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
