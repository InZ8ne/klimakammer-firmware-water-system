#include "Arduino.h"
#include "Wire.h"

const char I2C_ADDR = 0x55; //Set to desired i2c-adress
#undef DEBUG    //Define for various debug outputs (#undef to disable) - !!!ENABLING SLOWS DOWN CODE SIGNIFICANTLY!!!

#define Pump 0x12
#define Stepper 0x13
#define Sensor 0x14

//Pins for Pump
#define PWMPINForward 21
#define PWMPINReverse 19
#define nFaultPump 18
#define IPropi 27

//Pins for Pressure Sensing
#define ADCPinPressure 26

//Stepper
#define nFaultStepper 17 // use GPIO17 as nFault Pin for the Stepper
#define WIRE1_SDA       10  // Use GPIO10 as I2C1 SDA
#define WIRE1_SCL       11  // Use GPIO11 as I2C1 SCL
#define StepperControllerAddress 0x60 // Default 7bit i2c address of the DRV8847S
#define IC1ControlRegister 0x01 // Address of the i2c IC1 Control Register of the DRV8847S
#define Config4Bit 0x04 // Definition for 4Pin Interface Mode and using the i2c bits for steering


#ifndef esp32dev  
  arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);
#endif

#define BUILTIN_LED 25 //GPIO of BUILTIN_LED for pico
#ifdef esp32dev
  #undef BUILTIN_LED
  #define BUILTIN_LED 2 //GPIO of BUILTIN_LED for esp32dev
#endif


void sendData(float data1, float data2);  //Function to send data back to the master
void sendData(int data1, int data2);  //Overload to accept int as argument
void sendData(char data1, char data2);  //Overload to accept char as argument
void onRequest(); //Code to execute when master requests data from the slave
void onReceive(int len);  //Code to execute when master sends data to the slave

#pragma region sendData

void sendData(float data1 = 0, float data2 = 0){  //Function to send data back to the master
  //Pointer to the float
  uint8_t *bytePointer1 = reinterpret_cast<uint8_t*>(&data1);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire1.write((*bytePointer1));
      bytePointer1++;
  }

  //Pointer to the second float
  uint8_t *bytePointer2 = reinterpret_cast<uint8_t*>(&data2);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire1.write((*bytePointer2));
      bytePointer2++;
  }
}

void sendData(int data1 = 0, int data2 = 0){ //Overload to accept int as argument
  sendData((float)data1, (float)data2);
}

void sendData(char data1 = 0, char data2 = 0){  //Overload to accept char as argument
  sendData((float)data1, (float)data2);
}

#pragma endregion

#ifdef DEBUG
void blink(){
  for (char i = 0; i<10; i++){
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
  }
}
#endif

void onRequest(){ //Code to execute when master requests data from the slave
  #ifdef DEBUG
  Serial.println("OnRequest");
  Serial.println(Wire.peek());
  blink();
  #endif
  char module = Wire.read();  //Read from which sensor/module the master wants data
  switch(module){
    case Pump:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called

      break;
    case Stepper:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      break;
    case Sensor:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
}

void onReceive(int len){
  #ifdef DEBUG
  Serial.println("OnReceive");
  blink();
  #endif
  //Code to execute when master sends data to the slave
  char module = Wire1.read();  //Read from which sensor/module the master wants to change
  char data = Wire1.read();  //Read the data the master wants to send
  switch(module){
    case Pump:
      #ifdef DEBUG
        Serial.println("Pump called");
      #endif
      //Code to execute when Pump is being called
      analogWrite(PWMPINForward, data); //Sets the speed of the DC Pump based on what the master wants
      break;
    case Stepper:
      #ifdef DEBUG
        Serial.println("Stepper called");
      #endif
      //Code to execute when Stepper is being called
      break;
    case Sensor:
      #ifdef DEBUG
        Serial.println("Sensor called");
      #endif
      //Code to execute when Sensor is being called
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
}

void transmitToStepperController(){
  char slaveAddress = StepperControllerAddress << 1; //Shifting the 7bit Slave Address by 1 bit to signify a write
  Wire.beginTransmission(slaveAddress); // Starting the Transmission with the slave
  Wire.write(IC1ControlRegister); //Transmit first 8 Bits for register-address
  /*TODO: zu fahrender Winkel nach vorne oder hinten berechnen und INx bits rotieren lassen.
  Aktueller Winkel speichern.
  Routinen für Vorwärts und Rückwärtsfahren.*/
}

void setupPump(){
  pinMode(PWMPINForward, OUTPUT); //Sets GPIO21 as an PWM output
  pinMode(PWMPINReverse, OUTPUT); //Sets GPIO19 as an PWM output
  pinMode(nFaultPump, INPUT); //Sets the Fault-Sense Pin as an input
  pinMode(IPropi, INPUT); //Sets the Current Sensing Pin as an input

  analogWrite(PWMPINForward, 0); //Initializes PWM on the PWM Pins with duty cycle 0
  analogWrite(PWMPINReverse, 0);
}

void setupPressureSensor(){
  pinMode(ADCPinPressure, INPUT); // Sets the Pin to measure WaterPressure as an Input
}

void setupStepper(){
  pinMode(nFaultStepper, INPUT); //Sets the Fault-Sense Pin as an input
}

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
  pinMode(BUILTIN_LED, OUTPUT);
  #endif
  Serial.begin(115200);

  setupPump(); //Setup Raspi Pins for the Pump
  setupPressureSensor(); //setup Raspi Pins for the pressure sensor
  setupStepper(); // setup Raspi pins for the StepperController

  Wire1.onReceive(onReceive);  //Function to be called when a master sends data to the slave
  Wire1.onRequest(onRequest);  //Function to be called when a master requests data from the slave
  Wire1.begin((uint8_t)I2C_ADDR);  //Register this device as a slave on the i2c-bus (on bus 1)
  Wire.begin(); //Set this device as a master on the i2c-bus (bus 0)

}

void loop() {
  // put your main code here, to run repeatedly:

}
