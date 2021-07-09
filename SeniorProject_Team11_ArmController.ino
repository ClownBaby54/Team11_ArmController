/*
 * Team 11
 * EMG Muscle Memory
 * Description: This code is for the Robot Arm Controller
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "VarSpeedServo.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
/*Pin assignments */
//Pin 13 -> SCK  (52)
//Pin 12 -> MISO (50)
//Pin 11 -> MOSI (51)
//Pin  8 -> CSN  
//Pin  7 -> CE
//Unused -> IRQ

VarSpeedServo myservo1;  // create servo object to control a servo               
VarSpeedServo myservo2;  // create servo object to control a servo 
VarSpeedServo myservo3;  // create servo object to control a servo 

const int servoPin1 = 9; // the digital pin used for the first servo
const int servoPin2 = 6; // the digital pin used for the second servo
const int servoPin3 = 5; // the digital pin used for the first servo

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = {"10dar", "30dar"};

// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

int letssee = 10;

struct PayloadStruct {
  char message[13];          // only using 13 characters for TX & ACK payloads
  uint8_t counter;
};
PayloadStruct payload;

void setup() {
  Serial.begin(115200);
  myservo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object
  myservo1.write(0,20,true); // set the intial position of the servo, as fast as possible, run in background
  myservo2.attach(servoPin2);  // attaches the servo on pin 9 to the servo object
  myservo2.write(0,20,true);  // set the intial position of the servo, as fast as possible, wait until done
  myservo3.attach(servoPin3);  // attaches the servo on pin 9 to the servo object
  myservo3.write(0,20,true);  // set the intial position of the servo, as fast as possible, wait until done

//ensure that the select is turned on to enable commmunication
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

    // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    while (1) {} // hold in infinite loop
  }
  
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  Serial.println("Connected");
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);     // RF24_PA_MAX is default.

  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  radio.startListening();                                     // put radio in RX mode
} // void setup loop

void loop() 
{
  
    int Servo1Angle;
    int Servo2Angle;
    int Servo3Angle;
    
    uint8_t pipe;

    if (radio.available(&pipe)) 
    {                    // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
      PayloadStruct received;
      radio.read(&received, sizeof(received));       // get incoming payload

      radio.stopListening();                                      // put radio in TX mode
      radio.write(&received, sizeof(received));    // transmit to usb dongle for data display. We dont care if it doesnt receieve it since we'll see it
      radio.startListening();
      
      //convert servo angles from incoming EMG data to 180 degree scale
      if(received.message[1] == '.')
      {
        Servo1Angle = (float)(received.message[0] - '0')/99*180;
      }
      else
      {
        Servo1Angle = (float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*180;
      }
      
//      if(received.message[3] == '.')
//      {
//        Servo2Angle = (float)(received.message[2] - '0')/99*180;
//      }
//      else
//      {
//        Servo2Angle = (float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*180;
//      }
//
//      if(received.message[5] == '.')
//      {
//        Servo3Angle = (float)(received.message[4] - '0')/99*180;
//      }
//      else
//      {
//        Servo3Angle = (float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180;
//      }
      
      Serial.println(received.message[0]);
//      Serial.println(Servo1Angle);   
      
      myservo1.write(Servo1Angle,50,false); // set the position of the servo, as fast as possible, run in background
//      myservo2.write(Servo2Angle,50,false); // set the position of the servo, as fast as possible, run in background
//      myservo3.write(Servo3Angle,50,false); // set the position of the servo, as fast as possible, run in background
      
    }// if (radio.available(&pipe)) 

} // loop
