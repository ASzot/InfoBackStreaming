#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


/////////////////////////
// Constant definitions. 
///////////////////////

#define MOTOR_0 2
#define MOTOR_1 3
#define MOTOR_2 4
#define MOTOR_3 5
#define MOTOR_4 6
#define MOTOR_5 7
#define MOTOR_6 8
#define BREAK_MOTOR 9

// Vibrate time of the delay motor.
#define WAIT_TIME 700

// Vibrate time of each motor.
#define MOTOR_TIME 400

// The delay time between the motor vibrations.
#define DELAY_TIME 200





// Helper function to vibrate a set of motors. 
void vibrate_motor(int vibratePin)
{
  // Start the motors vibrating.
  
  // Display a message.
  Serial.print("Vibrating motor ");
  Serial.println(vibratePin);
  
  digitalWrite(vibratePin, HIGH);

  // Keep the motors vibrating.
  delay(MOTOR_TIME);

  Serial.print("Turning off motor ");
  Serial.println(vibratePin);
  
  digitalWrite(vibratePin, LOW);

  delete ledPins;
}

void vibrate_break()
{
  digitalWrite(BREAK_MOTOR, HIGH);

  delay(WAIT_TIME);

  digitalWrite(BREAK_MOTOR, LOW);
}

void setup_motors(void)
{ 
  // Set all of the digital IOs to output.
  int[] motors = [ MOTOR_0, MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4, MOTOR_5, MOTOR_6, BREAK_MOTOR ];

  for (int i = 0; i < 8; ++i)
  {
    pinMode(motors[i], OUTPUT);
    digitalWrite(motors[i], LOW);
  }
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  setup_motors();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
  char n, inputs[BUFSIZE+1];

  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }

  // Echo received data
  while (ble.available())
  {
    int c = ble.read();

    Serial.println((char)c);

    int[] motorsToVibrate = ConvertCharToMotorSeq(c);

    // Turn on the corresponding motors.
    for (int i = 0; i < 2; ++i) 
    {
      vibrate_motor(motorsToVibrate[i]);
    }
    
    delay(DELAY_TIME);

    // Vibrate the break motor.
    vibrate_break();
  }
}

int[] ConvertCharToMotorSeq(char c) 
{
  int[] motorSeq = [ -1, -1];
  switch (c) 
  {
    case 'A':
    case 'a':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_0;
      break;
    case 'B':
    case 'b':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_1;
      break;
    case 'C':
    case 'c':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_2;
      break;
    case 'D':
    case 'd':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_3;
      break;
    case 'E':
    case 'e':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_4;
      break;
    case 'F':
    case 'f':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_5;
      break;
    case 'G':
    case 'g':
      motorSeq[0] = MOTOR_0;
      motorSeq[1] = MOTOR_6;
      break;
    case 'H':
    case 'h':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_0;
      break;
    case 'I':
    case 'i':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_1;
      break;
    case 'J':
    case 'j':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_2;
      break;
    case 'K':
    case 'k':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_3;
      break;
    case 'L':
    case 'l':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_4;
      break;
    case 'M':
    case 'm':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_5;
      break;
    case 'N':
    case 'n':
      motorSeq[0] = MOTOR_1;
      motorSeq[1] = MOTOR_6;
      break;
    case 'O':
    case 'o':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_0;
      break;
    case 'P':
    case 'p':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_1;
      break;
    case 'Q':
    case 'q':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_2;
      break;
    case 'R':
    case 'r':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_3;
      break;
    case 'S':
    case 's':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_4;
      break;
    case 'T':
    case 't':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_5;
      break;
    case 'U':
    case 'u':
      motorSeq[0] = MOTOR_2;
      motorSeq[1] = MOTOR_6;
      break;
    case 'V':
    case 'v':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_0;
      break;
    case 'W':
    case 'w':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_1;
      break;
    case 'X':
    case 'x':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_2;
      break;
    case 'Y':
    case 'y':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_3;
      break;
    case 'Z':
    case 'z':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_4;
      break;
    case '$':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_5;
      break;
    case '#':
      motorSeq[0] = MOTOR_3;
      motorSeq[1] = MOTOR_6;
      break;
    case '.':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_0;
      break;
    case ',':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_1;
      break;
    case '!':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_2;
      break;
    case '*':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_3;
      break;
    case '+':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_4;
      break;
    case '-':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_5;
      break;
    case '/':
      motorSeq[0] = MOTOR_4;
      motorSeq[1] = MOTOR_6;
      break;
    case '1':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_0;
      break;
    case '2':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_1;
      break;
    case '3':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_2;
      break;
    case '4':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_3;
      break;
    case '5':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_4;
      break;
    case '6':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_5;
      break;
    case '7':
      motorSeq[0] = MOTOR_5;
      motorSeq[1] = MOTOR_6;
      break;
    case '8':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_0;
      break;
    case '9':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_1;
      break;
    case '0':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_2;
      break;
    case '(':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_3;
      break;
    case ')':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_4;
      break;
    case '\"':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_5;
      break;
    case ' ':
      motorSeq[0] = MOTOR_6;
      motorSeq[1] = MOTOR_6;
      break;
  }

  return motorSeq;
}


