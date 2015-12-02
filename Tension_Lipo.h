/*
   Naza2HoTT
   Ziege-One
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328
 
 */
#include "Arduino.h"

// Coefficient calculation measures based on resistances 
// COEF = (VREF/1024) / (R2/(R1 + R2)) 
// VREF = 5V (internal Creference) 5/1024=0.0048828125
// VREF = 1,1V (reference interne) 1,1/1024=0.0010742
//
// I USE THE HARDWARE SCHEME OF THYZOON
//
#define COEF_LIPO1 0.0049656  // R11 = 3.83K     R21 = 1.0K
#define COEF_LIPO2 0.0093464  // R12 = 7.87K     R22 = 1.0K
#define COEF_LIPO3 0.0132308  // R13 = 11.8K     R23 = 1.0K 
#define COEF_LIPO4 0.0174344  // R14 = 15.8K     R24 = 1.0K
#define COEF_LIPO5 0.0        // R15 = 0K        R25 = 0K 
#define COEF_LIPO6 0.0        // R16 = 0K        R26 = 0K 

/*
#define COEF_LIPO1 0.0048828  // R11 = 2.15K     
#define COEF_LIPO2 0.0087209  // R12 = 1.69K     R22 = 2.15K 
#define COEF_LIPO3 0.0127862  // R13 = 3.48K     R23 = 2.15K 
#define COEF_LIPO4 0.0170558  // R14 = 5.36K     R24 = 2.15K 
#define COEF_LIPO5 0.0211210  // R15 = 7.15K     R25 = 2.15K 
#define COEF_LIPO6 0.0255269  // R16 = 9.09K     R26 = 2.15K 
*/
/* I use the hardware scheme of Thyzoon but if you want to choose the scheme of Sylvain http://sylvainreynaud.blogspot.fr/ use these constants:
#define COEF_LIPO1 0.0048828  // R1 = 0K     
#define COEF_LIPO2 0.0097676  // R1= 2.21K     R2 = 2.209K 0.4999
#define COEF_LIPO3 0.0153644  // R1= 4.71K     R2 = 2.194K 0.3178
#define COEF_LIPO4 0.0202271  // R1= 6.90K     R2 = 2.196K 0.2414
#define COEF_LIPO5 0.0        // R1 = 0K       R2 = 0K 
#define COEF_LIPO6 0.0        // R1 = 0K       R2 = 0K 
*/

class TensionLipo{
public:
  void begin(); 
  float Tension_Lipo1 ();
  float Tension_Lipo2 ();
  float Tension_Lipo3 ();
  float Tension_Lipo4 ();
  float Tension_Lipo5 ();
  float Tension_Lipo6 ();
  float Lipo_Mini (uint8_t nb_Lipo,float L1,float L2,float L3,float L4,float L5,float L6);
  uint8_t Detect ();
  
  private:
  uint16_t mesure(uint8_t pin);
};

