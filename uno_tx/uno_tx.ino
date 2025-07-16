#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define s_version "U.2025.07.16.1"

# define UP_BTN 2
# define RIGHT_BTN 3
# define DOWN_BTN 4
# define LEFT_BTN 5
# define EEE_BTN 6
# define FFF_BTN 7

# define Joy_BTN 8
# define Joy_X A0
# define Joy_Y A1

int bottons[]={UP_BTN, RIGHT_BTN, DOWN_BTN, LEFT_BTN, EEE_BTN, FFF_BTN, Joy_BTN};
 
#define CE_PIN 9
#define CSN_PIN 10

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
 
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
 
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
 
// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role
 
// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
//float payload = 0.0;
char c_payload;
 
void setup() {

    for(int i=0;i<7;i++) 
    pinMode(bottons[i],INPUT);

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
 
  Serial.print("Version");
  Serial.println(s_version);

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
 
  // print example's introductory prompt
  Serial.println(F("RF24/examples/GettingStarted"));
 /*
  // To set the radioNumber via the Serial monitor on startup
  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  while (!Serial.available()) {
    // wait for user input
  }
  char input = Serial.parseInt();
  radioNumber = input == 1;
  */
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);
 
  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
 
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
 
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(char));  // float datatype occupies 4 bytes
 
  // set the TX address of the RX node for use on the TX pipe (pipe 0)
  radio.stopListening(address[radioNumber]);  // put radio in TX mode
 
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  
  // For debugging info
   printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
//  radio.printPrettyDetails(); // (larger) function that prints human readable data
 
}  // setup
/*
  Serial.println("Send the character 'f' to move forward");
  Serial.println("Send the character 'b' to move backward");
  Serial.println("Send the character 'l' to spin to the left");
  Serial.println("Send the character 'r' to spin to the right");
  Serial.println("Send the character 'q' to drive to forward left");
  Serial.println("Send the character 'w' to drive to forward right");
  Serial.println("Send the character 'a' to drive to backward left");
  Serial.println("Send the character 's to drive to backward right");
*/
 
void loop() {
  c_payload = 0;

  int joy_X = analogRead(Joy_X);
  int joy_Y = analogRead(Joy_Y);

  Serial.print(" Joy_X:"); Serial.print(joy_X);
  Serial.print(" Joy_Y:"); Serial.println(joy_Y);

  if (joy_Y > 900){
    if (joy_X > 900)
        c_payload = 'w'; // forward right
    else
      if (joy_X < 100)
        c_payload = 'q'; // forward left
      else
        c_payload = 'f'; // just forward
  }
  else{
    if (joy_Y < 100){
      if (joy_X > 900)
          c_payload = 's'; // backward right
      else
        if (joy_X < 100)
          c_payload = 'a'; // backward left
        else
        c_payload = 'b'; // just backward
    }
    else{// Y is between 100 and 900
      if (joy_X > 900)
          c_payload = 'r'; // spin left
      else
        if (joy_X < 100)
          c_payload = 'l'; // spin right
        else
          c_payload = 'z';// zero
    }
  }

  if (digitalRead(UP_BTN) == 0)// 0 means pressed
    c_payload = 'u'; // increase current
  else
    if (digitalRead(DOWN_BTN) == 0)// 0 means pressed
      c_payload = 'd'; // decrease current
    else
      if (digitalRead(LEFT_BTN) == 0)// 0 means pressed
        c_payload = 'g'; // get params
      else
        if (digitalRead(RIGHT_BTN) == 0)// 0 means pressed
          c_payload = 'c'; // clear errors
        else
          if (digitalRead(Joy_BTN) == 0)// 0 means pressed
            c_payload = 'i'; // idle
          else
          if (digitalRead(EEE_BTN) == 0)// 0 means pressed
            c_payload = 'h'; // hold position
          

    // This device is a TX node
 
  unsigned long start_timer = micros();                // start the timer
  bool report = radio.write(&c_payload, sizeof(char));  // transmit & save the report
  unsigned long end_timer = micros();                  // end the timer
 
  if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(c_payload);  // print payload sent
      //payload += 0.01;          // increment float payload
  } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
      //radio.printPrettyDetails();
  }
 
    // to make this example readable in the serial monitor
    delay(100);  // slow transmissions down by 1 second
}  // loop