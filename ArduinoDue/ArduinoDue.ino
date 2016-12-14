/*
 * ARDUINO GATEWAY : create connection between the Pi and the sensors
 */

#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x40    // address of the slave device

// pin definitions
#define RELAY_PIN  6
#define SOIL_PIN_A  1
#define SOIL_PIN_B  7

// these constants represents the position of the A and B plants. Should be set to the actual project.
#define MAX_SERVO_POS  165
#define MIN_SERVO_POS  15

Servo plantServo;
int soilValueA;
int soilValueB;
byte outputBuffer[2];
byte inputCommand;

// reads the plant A and map the value to the range of the raspberry's adc
int soilReadA(){
  int sensorValueA = analogRead(SOIL_PIN_A);
  return map(sensorValueA,0,4095,0,32768); //arm arduino
  //return map(sensorValueA,0,1023,0,32768); //avr arduino
}

// reads the plant B and map the value to the range of the raspberry's adc
int soilReadB(){
  int sensorValueB = analogRead(SOIL_PIN_B);
  return map(sensorValueB,0,4095,0,32768); //arm arduino
  //return map(sensorValueB,0,1023,0,32768); //avr arduino
}

/*
 * I2c procedures
 */

//  write
void receiveEvent(int howMany) {
  int result;
  while(1 <= Wire.available()){
    inputCommand = Wire.read();
    switch(inputCommand){
      case 0x06:
        digitalWrite(RELAY_PIN, HIGH);  // turn relay on
        break;
      case 0x01:
        digitalWrite(RELAY_PIN, LOW);   // turn relay off
        break;
      case 0x02:
        plantServo.write(MAX_SERVO_POS); // turn servo to direction
        break;
      case 0x03:
        plantServo.write(MIN_SERVO_POS);
        break;
      case 0x04:
        result = soilReadA();           // read the humidity and creates a byte array of it
        outputBuffer[0] = highByte(result);
        outputBuffer[1] = lowByte(result);       
        break;
      case 0x05:
        result = soilReadB();
        outputBuffer[0] = highByte(result);
        outputBuffer[1] = lowByte(result);       
        break;
      default:
        // nothing to do
        break;
    }
  }
}

//  read
void requestEvent() {
  switch(inputCommand){
    case 0x04:                    // sends the output buffer
      Wire.write(outputBuffer,2);
      break;
    case 0x05:
      Wire.write(outputBuffer,2);
      break;
    default:
      // no answer
      break;
  }
}

void setup() {
  plantServo.attach(9);       // servo uses pin 9
  pinMode(RELAY_PIN,OUTPUT);
  pinMode(SOIL_PIN_A,INPUT);
  pinMode(SOIL_PIN_B,INPUT);
  
  analogReadResolution(12);   // set the resolution on the arduino due to 12 bit, arm arduino only
                              
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);  // I2c settings
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // empty
}



