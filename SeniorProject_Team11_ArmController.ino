/*
 * Team 11
 * EMG Muscle Memory
 * Robot Arm Controller
 * 
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "VarSpeedServo.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

VarSpeedServo myservo1;  // create servo object to control a servo               
VarSpeedServo myservo2;
VarSpeedServo myservo3;  // create servo object to control a servo 

const int servoPin1 = 9; // the digital pin used for the first servo
const int servoPin2 = 6; // the digital pin used for the second servo
const int servoPin3 = 5; // the digital pin used for the first servo

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = {"10dar", "20dar"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
// to use different addresses on a pair of radios, we need a variable to

// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

int letssee = 10;

// For this example, we'll be using a payload containing
// a string & an integer number that will be incremented
// on every successful transmission.
// Make a data structure to store the entire payload of different datatypes
struct PayloadStruct {
  char message[13];          // only using 13 characters for TX & ACK payloads
  uint8_t counter;
};
PayloadStruct payload;

void setup() {

  
//  Serial.begin(115200);
  myservo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object
  myservo1.write(0,50,true); // set the intial position of the servo, as fast as possible, run in background
  myservo2.attach(servoPin2);  // attaches the servo on pin 9 to the servo object
  myservo2.write(0,50,true);  // set the intial position of the servo, as fast as possible, wait until done
  myservo3.attach(servoPin3);  // attaches the servo on pin 9 to the servo object
  myservo3.write(0,50,true);  // set the intial position of the servo, as fast as possible, wait until done

    // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    while (1) {} // hold in infinite loop
  }
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);     // RF24_PA_MAX is default.

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  // setup the ACK payload & load the first response into the FIFO
  memcpy(payload.message, "12 ", 3);                       // set the payload message
  // load the payload for the first received transmission on pipe 0
  radio.writeAckPayload(1, &payload, sizeof(PayloadStruct));

  radio.startListening();                                     // put radio in RX mode
}

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

      // save incoming counter & increment for next outgoing
      payload.counter = received.counter + 1;
      letssee = letssee+1;
      sprintf(payload.message, "%d", letssee);
      // load the payload for the first received transmission on pipe 0
      radio.writeAckPayload(1, &payload, sizeof(payload));

      //convert servo angles from incoming EMG data to 180 degree scale
      if(received.message[1] == '.')
      {
        Servo1Angle = (float)(received.message[0] - '0')/99*180;
      }
      else
      {
        Servo1Angle = (float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*180;
      }
//      Serial.println(received.message[0]);
//      Serial.println(Servo1Angle);
      
//      Servo1Angle = ((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*180;
//      Servo2Angle = ((received.message[1] - '0')*10 + (received.message[2] - '0'))/99*180;
//      Servo3Angle = ((received.message[3] - '0')*10 + (received.message[4] - '0'))/99*180;
      
      myservo1.write(Servo1Angle,50,false); // set the position of the servo, as fast as possible, run in background
//      myservo2.write(Servo2Angle,50,false); // set the position of the servo, as fast as possible, run in background
//      myservo3.write(Servo3Angle,50,false); // set the position of the servo, as fast as possible, run in background
    }

} // loop
