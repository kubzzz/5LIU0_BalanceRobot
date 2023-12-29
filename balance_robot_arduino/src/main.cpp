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
#include <Arduino.h>
#include <ICM42688.h>     // ICM42688 IMU Library
//#include <TMC2209.h>      // TMC2209 stepper drivers Library
#include <AccelStepper.h> // Accel Stepper Library 

#define LED_PIN 13 // Built in LED of Arduino
#define WIRE Wire1 // Wire=SDA,SCL Wire1=SDA1,SCL1

void print_IMU_data(void);

/* Making IMU object with I2C channel (Wire1) and address (0x68) */
ICM42688 IMU(WIRE, 0x68);

/* Making an motor object where pin(11)=STEP pin(12)=DIR */
AccelStepper motorR(1, 11, 12);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // begin serial communication to read data from the arduino
  while (!Serial) {}

  motorR.setEnablePin(8);
  motorR.setMaxSpeed(500);
  motorR.setSpeed(50);
  
  int status = IMU.begin();
  /* Check if the IMU is correctly working */
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelFS(ICM42688::gpm8);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroFS(ICM42688::dps500);
  
  // set output data rate to 12.5 Hz
  IMU.setAccelODR(ICM42688::odr12_5);
  IMU.setGyroODR(ICM42688::odr12_5);

  Serial.println("ax,ay,az,gx,gy,gz,temp_C");


}

void loop() {
  // put your main code here, to run repeatedly:

  motorR.runSpeed();

  
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
