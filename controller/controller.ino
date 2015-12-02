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

#define MOTOR_0 2
#define MOTOR_1 10
#define BREAK_MOTOR 11

#define PIN_COUNT 13

// The time in milliseconds each vibrator vibrates for. 
#define WAIT_TIME 1000

// Helper function to vibrate a set of motors. 
void vibrate_motors(int ledPins[])
{
  // Start the motors vibrating.
  for (int i = 0; i < PIN_COUNT; ++i)
  {
    if (ledPins[i] == 0)
      continue;
    
    // Display a message.
    Serial.print("Vibrating motor ");
    Serial.println(i);
    
    digitalWrite(i, HIGH);
  }

  // Keep the motors vibrating.
  delay(WAIT_TIME);

  // Turn the motors off.
  for (int i = 0; i < PIN_COUNT; ++i)
  {
    if (ledPins[i] == 0)
      continue;

    Serial.print("Turning off motor ");
    Serial.println(i);
    digitalWrite(i, LOW);
  }

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
  pinMode(MOTOR_0, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(BREAK_MOTOR, OUTPUT);

  // Set all of the motors to off by default.
  digitalWrite(MOTOR_0, LOW);
  digitalWrite(MOTOR_1, LOW);
  digitalWrite(BREAK_MOTOR, LOW);
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
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    int dataArray[5] = { -1 };
    int len = -1;

    switch (c)
    {
      case 'A':
      case 'a':
        dataArray[0] = 0;
        dataArray[1] = 1;
        break;
      case 'B':
      case 'b':
        dataArray[0] = 1;
        dataArray[1] = 0;
        dataArray[2] = 0;
        dataArray[3] = 0;
        break;
    }

    for (int i = 0; i < 5; ++i)
    {
      // End of transmitting the character.
      if (dataArray[i] == -1)
        break;

      // Turn on the corresponding motors.
      int motorPins[PIN_COUNT] = { 0 };
      if (dataArray[i] = 1)
        motorPins[MOTOR_0] = 1;
      else
        motorPins[MOTOR_1] = 1;

      vibrate_motors(motorPins);
    }

    //////////////////////////////////
    // Comment the vibrate_break(); and uncomment the delay(WAIT_TIME); line to pause for WAIT_TIME milliseconds between characters.
    // Do the opposite to vibrate the break motor located at pin BREAK_MOTOR.

    // vibrate the break motor.
    vibrate_break();

    // Or just wait.
    //delay(WAIT_TIME);
  }
}
