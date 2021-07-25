/*
 * Team 11
 * EMG Muscle Memory
 * Robot Arm Controller
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
const int servoPin2 = 6; // the digital pin used for the second servo
const int servoPin3 = 5; // the digital pin used for the first servo

Adafruit_NeoPixel pixels1(12, 3, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(12, 4, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels3(12, 10, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels4(12, 11, NEO_GRB + NEO_KHZ800);

void SetLED1(int j);
void SetLED2(int j);
void SetLED3(int j);
void SetLED4(int j);

void ClearLED1(int j);
void ClearLED2(int j);
void ClearLED3(int j);
void ClearLED4(int j);

#define S3Min 90; // This corresponds to the elbow servo at "straight"
#define S2Min 30; // This corresponds to the shoulder adduction/abduction servo at "straight down

float Servo1LEDVal;

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

  
  myservo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object
  myservo1.write(45,20,true); // set the intial position of the servo, as fast as possible, run in background
  myservo2.attach(servoPin2);  // attaches the servo on pin 6 the servo object
  myservo2.write(45,20,true);  // set the intial position of the servo, as fast as possible, wait until done
  myservo3.attach(servoPin3);  // attaches the servo on pin 5 to the servo object
  myservo3.write(45,20,true);  // set the intial position of the servo, as fast as possible, wait until done

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
  ClearLED1(0);
  ClearLED2(0);
  ClearLED3(0);
  ClearLED4(0);
//ensure that the select is turned on to enable commmunication
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
    // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    SetLED1(270);
    while (1) {} // hold in infinite loop
  }
  
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  Serial.println("Connected");
  
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  //radio.setPALevel(RF24_PA_LOW);     // RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_HIGH);     // RF24_PA_MAX is default.
  
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  radio.startListening();                                     // put radio in RX mode

}

void loop() 
{
  
    int Servo1Angle;
    int Servo2Angle;
    int Servo3Angle;
    int Sensor3Val;
    int Sensor4Val;
    
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

      //Sensor1
      if(received.message[1] == '.')
      {
        if(((Servo1Angle - ((float)(received.message[0] - '0')/99*180)) > 10) || ((Servo1Angle - ((float)(received.message[0] - '0')/99*180)) < - 10))
          {
            Servo1Angle = (float)(received.message[0] - '0')/99*180;
          }
      }
      else
      {
        if(((Servo1Angle - ((float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*180)) > 10) || ((Servo1Angle - ((float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*180)) < - 10))
        {
          Servo1Angle = (float)((received.message[0] - '0')*10 + (received.message[1] - '0'))/99*180;
        }
      }
      //Sensor2
      if(received.message[3] == '.')
      {
        if(((Servo2Angle - ((float)(received.message[2] - '0')/99*180)) > 10) || ((Servo2Angle - ((float)(received.message[2] - '0')/99*180)) < - 10))
        {
          Servo2Angle = (float)(received.message[2] - '0')/99*180;
        }
      }
      else
      {
        if(((Servo2Angle - ((float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*180)) > 10) || ((Servo2Angle - ((float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*180)) < - 10))
        {
          Servo2Angle = (float)((received.message[2] - '0')*10 + (received.message[3] - '0'))/99*180;
        }
      }
      //Sensor3
      if(received.message[5] == '.')
      {
        if(((Sensor3Val - ((float)(received.message[4] - '0')/99*180)) > 10) || ((Sensor3Val - ((float)(received.message[4] - '0')/99*180)) < - 10))
        {
          Sensor3Val = (float)(received.message[4] - '0')/99*180;
        }
      }
      else
      {
        if(((Sensor3Val - ((float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180)) > 10) || ((Sensor3Val - ((float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180)) < - 10))
        {
          Sensor3Val = (float)((received.message[4] - '0')*10 + (received.message[5] - '0'))/99*180;
        }
      }
      //Sensor4
      if(received.message[7] == '.')
      {
        if(((Sensor4Val - ((float)(received.message[6] - '0')/99*180)) > 10) || ((Sensor4Val - ((float)(received.message[6] - '0')/99*180)) < - 10))
        {
          Sensor4Val = (float)(received.message[6] - '0')/99*180;
        }
      }
      else
      {
        if(((Sensor4Val - ((float)((received.message[6] - '0')*10 + (received.message[7] - '0'))/99*180)) > 10) || ((Sensor4Val - ((float)((received.message[6] - '0')*10 + (received.message[7] - '0'))/99*180)) < - 10))
        {
          Sensor4Val = (float)((received.message[6] - '0')*10 + (received.message[7] - '0'))/99*180;
        }
      }

      //Arm rotation depends on difference of Sensor 3 and 4
      if((Sensor3Val - Sensor4Val) >= 0)                    //If sensor 3 is greater, go clockwise
      {
        Servo3Angle = (float)(Sensor3Val - Sensor4Val)/2+90;
      }
      else if((Sensor4Val - Sensor3Val) > 0)                //else if 4 is greater, go counter clockwise
      {
        Servo3Angle = (float)(-1*(Sensor4Val - Sensor3Val)/2)*90;
      }

      //Limits to prevent servos from breaking
      if(Servo1Angle > 85)
      {
        Servo1Angle = 85;
      }
      else if(Servo1Angle < 15)
      {
        Servo1Angle = 15;
      }

      if(Servo2Angle > 150)
      {
        Servo2Angle = 150;
      }
      else if(Servo2Angle < 35)
      {
        Servo2Angle = 35;
      }
      
      myservo1.write(Servo1Angle,50,false); // set the position of the servo, as fast as possible, run in background
      myservo2.write(Servo2Angle,50,false); // set the position of the servo, as fast as possible, run in background
      myservo3.write(Servo3Angle,50,false); // set the position of the servo, as fast as possible, run in background

      pixels1.clear();
      pixels2.clear();
      pixels3.clear();
      pixels4.clear();
      
      SetLED1(Servo1Angle);
      SetLED2(Servo2Angle);
      SetLED3(Sensor3Val);
      SetLED4(Sensor4Val);
      
    }

} // loop

void SetLED1(int j) {
  float test = j * 15;
  test = test / 120;
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

void SetLED2(int j) {
  float test = j * 15;
  test = test / 120;
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

void SetLED3(int j) {
  float test = j * 15;
  test = test / 120;
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

void SetLED4(int j) {
  float test = j * 15;
  test = test / 120;
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
