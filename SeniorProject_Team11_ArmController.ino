/*
 * Name:            Senior Project Team 11
 * File:            SeniorProject_Team11_ArmController.ino
 * Description:     This code is to be used to control the limb used in the EMG Controlled Robotic Limb Project.
 *                  It performs PWM control of NeoPixel LEDS and Servos, as well as SPI with an nrf24l01+ module.
 *                  This also utilizes VarSpeedServo.h to control the speed and acceleration of the Servos
 *
 */

#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "VarSpeedServo.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
//Pin 13 -> SCK  (52)
//Pin 12 -> MISO (50)
//Pin 11 -> MOSI (51)
//Pin  8 -> CSN  
//Pin  7 -> CE
//Unused -> IRQ

VarSpeedServo myservo1;  // create servo object to control a servo               
VarSpeedServo myservo2;
VarSpeedServo myservo3;  // create servo object to control a servo 

const int servoPin1 = 9; // the digital pin used for the first servo
const int servoPin2 = 5; // the digital pin used for the second servo
const int servoPin3 = 6; // the digital pin used for the first servo

Adafruit_NeoPixel pixels1(12, 3, NEO_GRB + NEO_KHZ800);     //Setting up the NeoPixels on Pins 3,4,10,11
Adafruit_NeoPixel pixels2(12, 4, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels3(12, 10, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels4(12, 11, NEO_GRB + NEO_KHZ800);

void SetLED1(float j);      //Functions used in setting the LEDS
void SetLED2(float j);
void SetLED3(float j);
void SetLED4(float j);

void ClearLED1(int j);      //Functions used in clearing 
void ClearLED2(int j);
void ClearLED3(int j);
void ClearLED4(int j);

float Servo1LEDVal;

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = {"10dar", "30dar"};      //Addresses are backwards compared to MSP432
// address is actually rad01 to the MSP432

// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

unsigned long previousMillis = 0;        // will store last time message was sent
const long interval = 5000;             // timeout for communication to harness RF module

struct PayloadStruct {
  char message[13];          // only using 13 characters for TX & ACK payloads
  uint8_t counter;
};

PayloadStruct payload;
PayloadStruct received;

//For adjusting ranges and resolution of servos
const int Servo1Offset = 85;
const int Servo1Scale  = -75;
const int Servo2Offset = 30;
const int Servo2Scale  = 90;
const int Servo3Offset = 0;
const int Servo3Scale  = 180;
const int Servo1Tol = 1;        //need to keep servos from twitching back and forth so keep a tolerance that the angles need to pass
const int Servo2Tol = 2;
const int Servo3Tol = 2;

void setup() {
  
  myservo1.write(85,20,false); // set the initial position of the servo, run in background
  myservo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object
  delay(2500);    
  myservo2.write(30,20,false);  // set the initial position of the servo, run in background
  myservo2.attach(servoPin2);  // attaches the servo on pin 6 the servo object
  delay(2500);
  myservo3.write(70,20,false);  // set the initial position of the servo, run in background
  myservo3.attach(servoPin3);  // attaches the servo on pin 5 to the servo object

  pixels1.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels2.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels3.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels4.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    
  for (int i = 3; i < 15; i++) { // For each pixel...
    if (i > 11) {
      i = i - 12;
      pixels1.setPixelColor(i, pixels1.Color(0 + i, 12 - i, 0 + i));
      pixels2.setPixelColor(i, pixels2.Color(0 + i, 12 - i, 0 + i));
      pixels3.setPixelColor(i, pixels3.Color(0 + i, 12 - i, 0 + i));
      pixels4.setPixelColor(i, pixels4.Color(0 + i, 12 - i, 0 + i));
      pixels1.show();
      pixels2.show();
      pixels3.show();
      pixels4.show();
      i = i + 12;
    }
    else {
      pixels1.setPixelColor(i, pixels1.Color(i, 12 - i, i));
      pixels2.setPixelColor(i, pixels2.Color(12 - i, 12 - i, i));
      pixels3.setPixelColor(i, pixels3.Color(i, i, 12 - i));
      pixels4.setPixelColor(i, pixels4.Color(i, 12 - i, i));
    }

    pixels1.show();
    pixels2.show();
    pixels3.show();
    pixels4.show();
    delay(100);
  }

  pixels1.clear();
  pixels2.clear();
  pixels3.clear();
  pixels4.clear();
    
  //ensure that the select is turned on to enable commmunication
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
    // initialize the transceiver on the SPI bus
  while (!radio.begin()) {
    SetLED1(1);
  }
  
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  // Set the PA Level high to get the range we need
  radio.setPALevel(RF24_PA_HIGH);     // RF24_PA_MAX is default.
  
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  radio.startListening();                                     // put radio in RX mode
  
  pinMode(13, INPUT);                                         //used to determine if Estop is pressed.

  //initializing the message string 
  received.message[0] = '0';
  received.message[1] = '0';
  received.message[2] = '0';
  received.message[3] = '0';
  received.message[4] = '0';
  received.message[5] = '0';
  received.message[6] = '0';
  received.message[7] = '0';
  received.message[8] = '0';
  received.message[9] = '0';
  received.message[10] = 0;
  received.message[11] = 0;
}

void loop() 
{
  
    int Servo1Angle;
    int Servo2Angle;
    int Servo3Angle;
    int Sensor3Val;
    int Sensor4Val;
 
    uint8_t pipe;
    
    unsigned long currentMillis = millis();         //grab running time
    
    if(!digitalRead(13))          //read value of ESTOP
    {
      myservo1.write(85,20,false); // return to resting position to avoid uncontrolled jumps
      myservo2.write(30,20,false);
      myservo3.write(70,20,false);  
    }

    //check for timeout of RF communication
    if (currentMillis - previousMillis >= interval) 
    {
       previousMillis = currentMillis;
       radio.stopListening();                                      // put radio in TX mode
       received.message[11] |= 0b00000001;                        //set error for communication
       radio.write(&received, sizeof(received));                  // transmit to usb dongle for data display. We dont care if it doesnt receive it since we'll see it visually
       radio.startListening();
    }
    else    //disable the error
    {
      received.message[11] &= ~0b00000001;
    }
    
    if (radio.available(&pipe)) 
    {                    // is there a payload? get the pipe number that recieved it
      previousMillis = currentMillis;                 //reset timeout counter
      uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
      radio.read(&received, sizeof(received));       // get incoming payload

      radio.stopListening();                                      // put radio in TX mode
      radio.write(&received, sizeof(received));    // transmit to usb dongle for data display. We dont care if it doesnt receieve it since we'll see it
      radio.startListening();
      
      //convert servo angles from incoming EMG data to custom degree scale

      //Sensor1
      if(received.message[1] == '.')    //is the second character just the decimal point?
      { //is our new angle within the tolerance specified?
        if(((Servo1Angle - ((float)(received.message[0] - '0')/99*Servo1Scale+Servo1Offset)) > Servo1Tol) || ((Servo1Angle - ((float)(received.message[0] - '0')/99*Servo1Scale+Servo1Offset)) < -1*Servo1Tol))
          { 
            Servo1Angle = (float)(received.message[0] - '0')/99*Servo1Scale+Servo1Offset;   //run conversion from 0-99 scale to custom angle scale
            if(Servo1Angle > Servo1Offset)                        //should be unecessary, but just in case, make sure angle always stays within bounds
            {
              Servo1Angle = Servo1Offset;
            }
            else if(Servo1Angle < Servo1Offset+Servo1Scale)
            {
              Servo1Angle = Servo1Offset+Servo1Scale;
            }
                      
            if(digitalRead(13))                     //if ESTOP not pressed
            {
              myservo1.write(Servo1Angle,40,false); // set the position of the servo, with max speed of 40, run in background
            }
            else    //or write to the pixels if no ESTOP
            {
              pixels1.clear();
              SetLED1((float)(1-((float)Servo1Angle-10)/75));
            }
          }
      }
      else
      {
        if(((Servo1Angle - ((float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*Servo1Scale+Servo1Offset)) > Servo1Tol) || ((Servo1Angle - ((float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*Servo1Scale+Servo1Offset)) < -1*Servo1Tol))
        {
          Servo1Angle = (float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*Servo1Scale+Servo1Offset;
          if(Servo1Angle > Servo1Offset)
          {
            Servo1Angle = Servo1Offset;
          }
          else if(Servo1Angle < Servo1Offset+Servo1Scale)
          {
            Servo1Angle = Servo1Offset+Servo1Scale;
          }
       
          if(digitalRead(13))
          {
            myservo1.write(Servo1Angle,40,false); // set the position of the servo, as fast as possible, run in background
          }
          else
          {
            pixels1.clear();
            SetLED1((float)(1-((float)Servo1Angle-10)/75));
          }
        }
      }
      //Sensor2
      if(received.message[3] == '.')
      {
        if(((Servo2Angle - ((float)(received.message[2] - '0')/99*Servo2Scale+Servo2Offset)) > Servo2Tol) || ((Servo2Angle - ((float)(received.message[2] - '0')/99*Servo2Scale+Servo2Offset)) < -1*Servo2Tol))
        {
          Servo2Angle = (float)(received.message[2] - '0')/99*Servo2Scale+Servo2Offset;
          
          if(Servo2Angle > Servo2Offset+Servo2Scale)
          {
            Servo2Angle = Servo2Offset+Servo2Scale;
          }
          else if(Servo2Angle < Servo2Offset)
          {
            Servo2Angle = Servo2Offset;
          }
             
          if(digitalRead(13))
          {
            myservo2.write(Servo2Angle,40,false); // set the position of the servo, as fast as possible, run in background
          }
          else
          {
            pixels2.clear();
            SetLED2((float)(Servo2Angle-Servo2Offset)/Servo2Scale);
          }
          
        }
      }
      else
      {
        if(((Servo2Angle - ((float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*Servo2Scale+Servo2Offset)) > Servo2Tol) || ((Servo2Angle - ((float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*Servo2Scale+Servo2Offset)) < -1*Servo2Tol))
        {
          Servo2Angle = (float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*Servo2Scale+Servo2Offset;

          if(Servo2Angle > Servo2Offset+Servo2Scale)
          {
            Servo2Angle = Servo2Offset+Servo2Scale;
          }
          else if(Servo2Angle < Servo2Offset)
          {
            Servo2Angle = Servo2Offset;
          }

          if(digitalRead(13))
          {
            myservo2.write(Servo2Angle,40,false); // set the position of the servo, as fast as possible, run in background
          }
          else
          {
            pixels2.clear();
            SetLED2((float)(Servo2Angle-Servo2Offset)/Servo2Scale);
          }
        }
      }
      //Sensor3                     //Sensor 3 and 4 need to be used in conjunction, so servo angle isnt set here
      if(received.message[5] == '.')
      {
        if(((Sensor3Val - ((float)(received.message[4] - '0')/99*180)) > 5) || ((Sensor3Val - ((float)(received.message[4] - '0')/99*180)) < -5))
        {
          Sensor3Val = (float)(received.message[4] - '0')/99*180;
          
          if(!digitalRead(13))
          {
            pixels3.clear();
            SetLED3((float)Sensor3Val/180);
          }
        }
      }
      else
      {
        if(((Sensor3Val - ((float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180)) > 5) || ((Sensor3Val - ((float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180)) < - 5))
        {
          Sensor3Val = (float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180;

          if(!digitalRead(13))
          {
            pixels3.clear();
            SetLED3((float)Sensor3Val/180);
          }
        }
      }
      //Sensor4
      if(received.message[7] == '.')
      {
        if(((Sensor4Val - ((float)(received.message[6] - '0')/99*180)) > 5) || ((Sensor4Val - ((float)(received.message[6] - '0')/99*180)) < -5))
        {
          Sensor4Val = (float)(received.message[6] - '0')/99*180;

          if(!digitalRead(13))
          {
            pixels4.clear();
            SetLED4((float)Sensor4Val/180);
          }
        }
      }
      else
      {
        if(((Sensor4Val - ((float)((received.message[6] - '0')*10 + (received.message[7] - '0'))/99*180)) > 5) || ((Sensor4Val - ((float)((received.message[6] - '0')*10 + (received.message[7] - '0'))/99*180)) < -5))
        {
          Sensor4Val = (float)((received.message[6] - '0')*10 + (received.message[7] - '0'))/99*180;
   
          if(!digitalRead(13))
          {
            pixels4.clear();
            SetLED4((float)Sensor4Val/180);
          }
        }
      }

      //Arm rotation depends on difference of Sensor 3 and 4
      if((Sensor3Val - Sensor4Val) >= 0)                    //If sensor 3 is greater, go counter clockwise
      {
        Servo3Angle = (float)(Sensor3Val - Sensor4Val)/2+90;
      }
      else if((Sensor4Val - Sensor3Val) > 0)                //else if 4 is greater, go clockwise
      {
        Servo3Angle = (float)(-1*(Sensor4Val - Sensor3Val)/2)+90;
      }
      else
      {
        Servo3Angle = 90;
      }


      if(digitalRead(13))   // if not ESTOP, then we can write to servo 3
      {
        myservo3.write(Servo3Angle,40,false); // set the position of the servo, at a speed of 40, run in background
      }

      
    }

} // loop

/*----------------------------------------------------------------
 * void SetLED1(float j)
 *
 * Description: Sets LEDs to value specified. 1 is fully illuminated, 0 is off
 * Inputs: Number from 0 to 1
 * Outputs: None
----------------------------------------------------------------*/
void SetLED1(float j) {
  float test = (float)j * 15;
  for (int i = 3; i < 15; i++) { // For each pixel...
    if (i <= test) {
      if (i > 11) {
        i = i - 12;
        pixels1.setPixelColor(i, pixels1.Color(0, 0, 5));
        pixels1.show();
        i = i + 12;
      }
      else {
        pixels1.setPixelColor(i, pixels1.Color(0, 0, 5));
        pixels1.show();
      }
    }
    else {
//      ClearLED1(test);
      break;
    }
  }
}

/*----------------------------------------------------------------
 * void SetLED2(float j)
 *
 * Description: Sets LEDs to value specified. 1 is fully illuminated, 0 is off
 * Inputs: Number from 0 to 1
 * Outputs: None
----------------------------------------------------------------*/
void SetLED2(float j) {
  float test = (float)j * 15;
  for (int i = 3; i < 15; i++) { // For each pixel...
    if (i <= test) {
      if (i > 11) {
        i = i - 12;
        pixels2.setPixelColor(i, pixels2.Color(0, 0, 5));
        pixels2.show();
        i = i + 12;
        
      }
      else {
        pixels2.setPixelColor(i, pixels2.Color(0, 0, 5));
        pixels2.show();
      }
    }
    else {
//      ClearLED1(test);
      break;
    }
  }
}

/*----------------------------------------------------------------
 * void SetLED3(float j)
 *
 * Description: Sets LEDs to value specified. 1 is fully illuminated, 0 is off
 * Inputs: Number from 0 to 1
 * Outputs: None
----------------------------------------------------------------*/
void SetLED3(float j) {
  float test = (float)j * 15;
  for (int i = 3; i < 15; i++) { // For each pixel...
    if (i <= test) {
      if (i > 11) {
        i = i - 12;
        pixels3.setPixelColor(i, pixels3.Color(0, 0, 5));
        pixels3.show();
        i = i + 12;
      }
      else {
        pixels3.setPixelColor(i, pixels3.Color(0, 0, 5));
        pixels3.show();
      }
    }
    else {
//      ClearLED1(test);
      break;
    }
  }
}

/*----------------------------------------------------------------
 * void SetLED4(float j)
 *
 * Description: Sets LEDs to value specified. 1 is fully illuminated, 0 is off
 * Inputs: Number from 0 to 1
 * Outputs: None
----------------------------------------------------------------*/
void SetLED4(float j) {
  float test = (float)j * 15;
  for (int i = 3; i < 15; i++) { // For each pixel...
    if (i <= test) {
      if (i > 11) {
        i = i - 12;
        pixels4.setPixelColor(i, pixels4.Color(0, 0, 5));
        pixels4.show();
        i = i + 12;
      }
      else {
        pixels4.setPixelColor(i, pixels4.Color(0, 0, 5));
        pixels4.show();
      }
    }
    else {
//      ClearLED1(test);
      break;
    }
  }
}

/*----------------------------------------------------------------
 * void ClearLED1(int j)
 *
 * Description: Clears LEDS
 * Inputs: Number of pixels to turn off
 * Outputs: None
----------------------------------------------------------------*/
void ClearLED1(int j) {
  if (j == 0) {
    for (int i = 0; i < 12; i++) { // For each pixel...
      pixels1.setPixelColor(i, pixels1.Color(0, 0, 0));
      pixels1.show();
    }
  }
  else {
    for (int i = j + 1; i < 12; i++) { // For each pixel...
      pixels1.setPixelColor(i, pixels1.Color(0, 0, 0));
      pixels1.show();
    }
  }
}

/*----------------------------------------------------------------
 * void ClearLED2(int j)
 *
 * Description: Clears LEDS
 * Inputs: Number of pixels to turn off
 * Outputs: None
----------------------------------------------------------------*/
void ClearLED2(int j) {
  if (j == 0) {
    for (int i = 0; i < 12; i++) { // For each pixel...
      pixels2.setPixelColor(i, pixels2.Color(0, 0, 0));
      pixels2.show();
    }
  }
  else {
    for (int i = j + 1; i < 12; i++) { // For each pixel...
      pixels2.setPixelColor(i, pixels2.Color(0, 0, 0));
      pixels2.show();
    }
  }
}

/*----------------------------------------------------------------
 * void ClearLED3(int j)
 *
 * Description: Clears LEDS
 * Inputs: Number of pixels to turn off
 * Outputs: None
----------------------------------------------------------------*/
void ClearLED3(int j) {
  if (j == 0) {
    for (int i = 0; i < 12; i++) { // For each pixel...
      pixels3.setPixelColor(i, pixels3.Color(0, 0, 0));
      pixels3.show();
    }
  }
  else {
    for (int i = j + 1; i < 12; i++) { // For each pixel...
      pixels3.setPixelColor(i, pixels3.Color(0, 0, 0));
      pixels3.show();
    }
  }
}

/*----------------------------------------------------------------
 * void ClearLED4(int j)
 *
 * Description: Clears LEDS
 * Inputs: Number of pixels to turn off
 * Outputs: None
----------------------------------------------------------------*/
void ClearLED4(int j) {
  if (j == 0) {
    for (int i = 0; i < 12; i++) { // For each pixel...
      pixels4.setPixelColor(i, pixels4.Color(0, 0, 0));
      pixels4.show();
    }
  }
  else {
    for (int i = j + 1; i < 12; i++) { // For each pixel...
      pixels4.setPixelColor(i, pixels4.Color(0, 0, 0));
      pixels4.show();
    }
  }
}
