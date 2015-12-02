/*
   Naza2HoTT
   Ziege-One
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328

 
 /////Pin Belegung////
 D0: 
 D1: 
 D2: 
 D3: RX / TX Softserial HoTT V4
 D4: 
 D5: 
 D6: 
 D7: 
 D8: 
 D9: 
 D10: 
 D11: 
 D12: 
 D13: LED, um die Kommunikation zu visualisieren
 
 A0: Lipo 1S
 A1: Lipo 2S
 A2: Lipo 3S
 A3: Lipo 4S
 A4: 
 A5: 
 
 
 */
 
// ======== Naza2HoTT  =======================================

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "Message.h"
#include "Tension_Lipo.h"
#include <inttypes.h>

// Green LED on pin 13
#define LEDPIN_PINMODE    pinMode (13, OUTPUT);
#define LEDPIN_SWITCH     PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF        PORTB &= ~(1<<5);
#define LEDPIN_ON         PORTB |= (1<<5);

//#define Debug                             // on off debuging

// Time interval [ms] for display updates:
const unsigned long DISPLAY_INTERVAL = 5000;
static unsigned long lastTime=0;  // in ms
unsigned long timer=millis();      // in ms


// Include Message.cpp functions for Init and Main program of LIPOMETER and GPS
GMessage message;


// ======== SETUP & CHECK =======================================
void setup()
{
  
  Serial.begin (115200); // 115200 FOR NAZA DJI GPS RX
  
  LEDPIN_PINMODE
  LEDPIN_ON
  delay(200);
  LEDPIN_OFF
  delay(200);
  LEDPIN_ON
  delay(200);
  LEDPIN_OFF
  delay(200);
  LEDPIN_ON
  delay(200);
  LEDPIN_OFF
  
  // Init GRAUPNER HOTT PROTOCOL
  message.init();
  
}

// ======== MAIN LOOP  =======================================
void loop()  {
  
    #ifdef Debug
      //FOR DEBUG ON SERIAL 115200
      timer=millis();
      if (timer-lastTime>DISPLAY_INTERVAL)  // if at least DISPLAY_INTERVAL ms have passed
      {
      message.debug();
      lastTime=timer;
      }
    #endif
  // No communication
  LEDPIN_OFF 
  
  // Sending and update GRAUPNER HOTT TELEMETRY AND DJI GPS
  message.main_loop();
}
