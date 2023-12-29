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

#define MOTOR_ENABLE 8
#define STEP_PIN_R 11
#define DIR_PIN_R 12
#define STEP_PIN_L 9
#define DIR_PIN_L 10

#define WIRE Wire1 // Wire=SDA,SCL Wire1=SDA1,SCL1

void print_IMU_data(void);

/* Making IMU object with I2C channel (Wire1) and address (0x68) */
ICM42688 IMU(WIRE, 0x68);

/* Making an motor object where pin(11)=STEP pin(12)=DIR */
AccelStepper motor_right(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);
AccelStepper motor_left(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);

int incomingByte = 0;
int number;
const unsigned int MAX_MESSAGE_LENGTH = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // begin serial communication to read data from the arduino
  while (!Serial) {}

  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH); // set HIGH to disable drivers set LOW to enable

  /* Set max velocity */
  motor_right.setMaxSpeed(3000);
  motor_left.setMaxSpeed(3000);

  /* Set acceleration */
  motor_right.setAcceleration(1);
  motor_left.setAcceleration(1);

  motor_left.setSpeed(-3000);
  motor_right.setSpeed(3000);
  
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

  while (Serial.available() > 0)
 {
   //Create a place to hold the incoming message
   static char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;
   //Read the next available byte in the serial receive buffer
   char inByte = Serial.read();
   //Message coming in (check not terminating character) and guard for over message size
   if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
   {
     //Add the incoming byte to our message
     message[message_pos] = inByte;
     message_pos++;
   }
   //Full message received...
   else
   {
    //Add null character to string
    message[message_pos] = '\0';
    //Print the message (or do other things)
    Serial.println(message);
    //Or convert to integer and print
    number = atoi(message);
    Serial.println(number);
    //Reset for the next message
    message_pos = 0;
   }
 }


  motor_left.setSpeed(-number);
  motor_right.setSpeed(number);

  motor_right.runSpeed();
  motor_left.runSpeed();

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
