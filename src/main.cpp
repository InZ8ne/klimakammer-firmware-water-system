#include "Arduino.h"
#include "Wire.h"

const char I2C_ADDR = 0x71; //Set to desired i2c-adress
#undef DEBUG    //Define for various debug outputs (#undef to disable) - !!!ENABLING SLOWS DOWN CODE SIGNIFICANTLY!!!

#define Pump 0x12
#define Stepper 0x13
#define Sensor 0x14

//Pump
#define PWMPINForward 21
#define PWMPINReverse 19
#define nFaultPump 18
#define IPropi 27

//Pressure Sensing
#define ADCPinPressure 26
#define MaxPressure 150
#define LSBPressure MaxPressure/255

//Stepper
#define nFaultStepper 17 // use GPIO17 as nFault Pin for the Stepper
#define WIRE1_SDA       10  // Use GPIO10 as I2C1 SDA
#define WIRE1_SCL       11  // Use GPIO11 as I2C1 SCL
#define StepperControllerAddress 0x60 // Default 7bit i2c address of the DRV8847S
#define IC1ControlRegister 0x01 // Address of the i2c IC1 Control Register of the DRV8847S
#define Config4Bit 0x04 // Definition for 4Pin Interface Mode and using the i2c bits for steering
#define Forward true
#define Backward false
#define MaxAngle 90
#define HalfStep 0.9
char currentPosition = 0;
char currentAngle = 0;
char module = 0;

#define BUILTIN_LED 25 //GPIO of BUILTIN_LED for pico
#ifdef esp32dev
  #undef BUILTIN_LED
  #define BUILTIN_LED 2 //GPIO of BUILTIN_LED for esp32dev
#endif

char data = 0;

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

char stepperNextState(char forward){
  char nextState = 0;
  char nextPosition = 0;
  bool In4 = false;
  bool In3 = false;
  bool In2 = false;
  bool In1 = false;
  char statesIn4[8] = {1,1,0,0,0,0,0,1};
  char statesIn3[8] = {0,0,0,1,1,1,0,0};
  char statesIn2[8] = {0,0,0,0,0,1,1,1};
  char statesIn1[8] = {0,1,1,1,0,0,0,0};

  if(forward){
    nextPosition = (currentPosition+1)%8;
    In4 = statesIn4[nextPosition];
    In3 = statesIn3[nextPosition];
    In2 = statesIn2[nextPosition];
    In1 = statesIn1[nextPosition];
  }
  else{
    nextPosition = (currentPosition+7)%8;
    In4 = statesIn4[nextPosition];
    In3 = statesIn3[nextPosition];
    In2 = statesIn2[nextPosition];
    In1 = statesIn1[nextPosition];
  }
  currentPosition = nextPosition;
  if (In4) {nextState |= (1<<3);}
  if (In3) {nextState |= (1<<2);}
  if (In2) {nextState |= (1<<1);}
  if (In1) {nextState |= 1;}
  return nextState;
}

void transmitToStepperController(int steps, char direction){
  char slaveAddress = StepperControllerAddress << 1; //Shifting the 7bit Slave Address by 1 bit to signify a write
  for (char i = 0; i<steps; i++){
    Wire.beginTransmission(slaveAddress); // Starting the Transmission with the slave
    Wire.write(IC1ControlRegister); //Transmit first 8 Bits for register-address
    char config = (stepperNextState(direction)<<3) | Config4Bit; //Generates the 8 bit word to programm the register
    Wire.write(config); //Writes the current configuration on the register
    Wire.endTransmission(); //ends the transmission
  }
}

int getWaterPressure(){
  int adcPressure = analogRead(ADCPinPressure);
  float actualPressure = adcPressure*LSBPressure;
  return (int)actualPressure;
}

void onRequest(){ //Code to execute when master requests data from the slave
  #ifdef DEBUG
  Serial.println("OnRequest");
  Serial.println(Wire.peek());
  blink();
  #endif
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
  module = Wire1.read();  //Read from which sensor/module the master wants to change
  if(!Wire1.available()){
    return;
  }
  data = Wire1.read();  //Read the data the master wants to send
  char direction;
  char newAngle;
  float steps;
  int waterPressure;
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
      direction = data>>7; //The MSB signifies the direction
      newAngle = data & 0b01111111; //The other 7 bit are the desired new angle
      steps = (int)(currentAngle-newAngle)/HalfStep; //calculate how many steps are needed
      currentAngle = newAngle; // Update the current Angle of the Stepper 
      transmitToStepperController(steps, direction); //Move the stepper to the new Position
      break;
    case Sensor:
      #ifdef DEBUG
        Serial.println("Sensor called");
      #endif
      //Code to execute when Sensor is being called
      waterPressure = getWaterPressure(); //get the value of water pressure
      // Code if pressure too high not defined yet
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
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
void setupWire1(){
  Wire1.setSDA(WIRE1_SDA); 
  Wire1.setSCL(WIRE1_SCL);
  Wire1.onReceive(onReceive);  //Function to be called when a master sends data to the slave
  Wire1.onRequest(onRequest);  //Function to be called when a master requests data from the slave
  Wire1.begin((uint8_t)I2C_ADDR);  //Register this device as a slave on the i2c-bus (on bus 1)
  Wire.begin(); //Set this device as a master on the i2c-bus (bus 0)
}

void setup() {
  #ifdef DEBUG
  pinMode(BUILTIN_LED, OUTPUT);
  #endif
  Serial.begin(115200);

  setupPump(); //Setup Raspi Pins for the Pump
  setupPressureSensor(); //setup Raspi Pins for the pressure sensor
  setupStepper(); // setup Raspi pins for the StepperController
  setupWire1(); //setup Raspi pins for the second I2C Interface

  analogWriteFreq(1000);  // PWM Frequency 1kHz
  analogWriteRange(255); //8bit PWM
}

void loop() {
  Serial.println((int)data);
  delay(500);
}
