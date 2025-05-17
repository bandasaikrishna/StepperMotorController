#include <Wire.h> //This is for i2C

int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float inAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
float delta=0;
const float gearRatio = 19.2;

float totalAngle = 0; //tared angle - based on the startup value
float outAngle = 0; //total absolute angular displacement
float prevAngle = 0; //for the display printing
//int speed = 5000;


#include <TMCStepper.h>

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, 0); 

#define RX2 16 // TMC2209/TMC2224 SoftwareSerial receive pin
#define TX2 17 // TMC2209/TMC2224 SoftwareSerial transmit pin

#include <PID_v1.h>      // For PID control

// PID tuning parameters
double Kp = 500.0;
double Ki = 0.1;
double Kd = 0.05;

// PID variables
double setpoint = 0.0;  // Target angle
double input = 0.0;     // Current angle
double output = 0.0;    // PID output

// PID controller instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


//I2C1
#define I2C_Slave_ADDR 0x08
#define I2C1_SDA 19
#define I2C1_SCL 18

volatile int16_t receivedAngle = 0;
bool newData = false;

void IRAM_ATTR receiveEvent(int howMany) {
  if(howMany == 2) {
    // For signed values, we need to properly reconstruct the 16-bit signed integer
    uint8_t msb = Wire1.read();
    uint8_t lsb = Wire1.read();
    receivedAngle = (int16_t)((msb << 8) | lsb);
    setpoint = (double)receivedAngle;
    newData = true;
    
    Serial.print("Received signed angle: ");
    Serial.println(receivedAngle);
  } else {
    while(Wire1.available()) Wire1.read(); // Flush buffer
  }
}

void IRAM_ATTR requestEvent() {
  // Send the outAngle (float) as bytes
  byte* byteData = (byte*)&outAngle;
  Wire1.write(byteData, sizeof(outAngle));
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(3000);
  Wire.begin(); //start i2C  
  Wire.setClock(800000L); //fast clock

  Serial.println("Search Magnet");

  checkMagnetPresence();

  delay(1000);

  SERIAL_PORT.begin(115200, SERIAL_8N1, RX2, TX2);      // HW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(1000);        // Set motor RMS current
  driver.microsteps(8);          // Set microsteps to 1/8th

  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  delay(1000);

    // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-6000, 6000);  // Adjust based on your motor's capabilities

  Wire1.begin(I2C_Slave_ADDR, I2C1_SDA, I2C1_SCL, 400000);
  Wire1.onReceive(receiveEvent);
  Wire1.onRequest(requestEvent);

  //driver.shaft(false);
  //driver.VACTUAL(speed);

}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor
    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }     
  //Status register output: 0 0 MD ML MH 0 0 0 
  Serial.println("Magnet found!");
  
  //Get the offset Angle in the first run
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left

  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  prevAngle = rawAngle * 0.087890625; 
  delay(1000);  
}

void ReadAngle()
{
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  inAngle = rawAngle * 0.087890625; 
  //Serial.print("In-Angle: ");
  //Serial.println(inAngle, 2); //absolute position of the encoder within the 0-360 circle
  //Serial.print("prevAngle: ");
  //Serial.println(prevAngle);


  delta = (inAngle-prevAngle);

  //Serial.print("delta: ");
  //Serial.println(delta);

  

  if (delta > 180) {
        // Wrapped around 360 → 0
        delta -= 360;
  } else if (delta < -180) {
        // Wrapped around 0 → 360
        delta += 360;
  }

  //if(fabs(delta)>1.5)
    //Serial.println("*********Angle missed**********************");


  totalAngle = totalAngle + delta;

  //Serial.print("totalAngle");
  //Serial.println(totalAngle);

  

  prevAngle = inAngle;

  outAngle = totalAngle / gearRatio;

  //Serial.print("OutAngle: ");
  //Serial.println(outAngle);


}

void loop() {

  ReadAngle();
  input = outAngle;

  if (Serial.available() > 0) {
    setpoint = Serial.parseFloat();
    while (Serial.available() > 0) {
      Serial.read(); // Clear the buffer
    }
    Serial.print("Ser New setpoint: ");
    Serial.println(setpoint);
  }

  if(newData)
  {
    Serial.print("I2C New setpoint: ");
    Serial.println(setpoint);
    newData = false;
  }

  myPID.Compute();
  //Serial Input for taking the angle.
  //PID controller command to speed command in driver.VACTUAL(speed) based on outAngle feedback

  // Determine direction based on output
  if (output > 0) {
    driver.shaft(false);  // Set direction forward
  } else {
    driver.shaft(true);   // Set direction reverse
  }

  // Set motor speed
  //Serial.print("V: ");
  //if(abs(output) >2000)
    //output =2000;
  Serial.println(output);

  driver.VACTUAL(abs(output));

}
