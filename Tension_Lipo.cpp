/*
   LIPOMETER v1.0
   http://johnlenfr.1s.fr
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328
 
 */

#include "Tension_Lipo.h"

void TensionLipo::begin(){
   //analogReference(DEFAULT); // alim reference. 5V
   analogReference(INTERNAL); // alim reference. 1.1V
   digitalWrite(A0, LOW); 
   pinMode(A0, INPUT);
   digitalWrite(A1, LOW); 
   pinMode(A1, INPUT);
   digitalWrite(A2, LOW); 
   pinMode(A2, INPUT);
   digitalWrite(A3, LOW); 
   pinMode(A3, INPUT);   
   digitalWrite(A4, LOW); 
   pinMode(A4, INPUT);    
   digitalWrite(A5, LOW); 
   pinMode(A5, INPUT); 
}

float TensionLipo::Tension_Lipo1 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  static float MV3 = 0.0;
  static float MV4 = 0.0;
  static float MV5 = 0.0;
  static float MV6 = 0.0;
  static float MV7 = 0.0;
  static float MV8 = 0.0;
  static float MV9 = 0.0;
  float MV10;
  float val;
  
  MV10 =  mesure(A0)* COEF_LIPO1;
  val = (MV1 + MV2 + MV3 + MV4 + MV5 + MV6 + MV7 + MV8 + MV9 + MV10) / 10; // filter (average of 10 samples)
  MV1 = MV2; // shift
  MV2 = MV3;
  MV3 = MV4;
  MV4 = MV5;
  MV5 = MV6;
  MV6 = MV7;
  MV7 = MV8;
  MV8 = MV9;
  MV9 = MV10;
  
  return val;
}

float TensionLipo::Tension_Lipo2 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  static float MV3 = 0.0;
  static float MV4 = 0.0;
  static float MV5 = 0.0;
  static float MV6 = 0.0;
  static float MV7 = 0.0;
  static float MV8 = 0.0;
  static float MV9 = 0.0;
  float MV10;
  float val;
  
  MV10 =  mesure(A1)* COEF_LIPO2;
  val = (MV1 + MV2 + MV3 + MV4 + MV5 + MV6 + MV7 + MV8 + MV9 + MV10) / 10; // filter (average of 10 samples)
  MV1 = MV2; // shift
  MV2 = MV3;
  MV3 = MV4;
  MV4 = MV5;
  MV5 = MV6;
  MV6 = MV7;
  MV7 = MV8;
  MV8 = MV9;
  MV9 = MV10;
  
  return val;  
}

float TensionLipo::Tension_Lipo3 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  static float MV3 = 0.0;
  static float MV4 = 0.0;
  static float MV5 = 0.0;
  static float MV6 = 0.0;
  static float MV7 = 0.0;
  static float MV8 = 0.0;
  static float MV9 = 0.0;
  float MV10;
  float val;
  
  MV10 =  mesure(A2)* COEF_LIPO3;
  val = (MV1 + MV2 + MV3 + MV4 + MV5 + MV6 + MV7 + MV8 + MV9 + MV10) / 10; // filter (average of 10 samples)
  MV1 = MV2; // shift
  MV2 = MV3;
  MV3 = MV4;
  MV4 = MV5;
  MV5 = MV6;
  MV6 = MV7;
  MV7 = MV8;
  MV8 = MV9;
  MV9 = MV10;
  
  return val;  
}

float TensionLipo::Tension_Lipo4 (){
  static float MV1 = 0.0; //4.20;
  static float MV2 = 0.0;
  static float MV3 = 0.0;
  static float MV4 = 0.0;
  static float MV5 = 0.0;
  static float MV6 = 0.0;
  static float MV7 = 0.0;
  static float MV8 = 0.0;
  static float MV9 = 0.0;
  float MV10;
  float val;
  
  MV10 =  mesure(A3)* COEF_LIPO4;
  val = (MV1 + MV2 + MV3 + MV4 + MV5 + MV6 + MV7 + MV8 + MV9 + MV10) / 10; // filter (average of 10 samples)
  MV1 = MV2; // shift
  MV2 = MV3;
  MV3 = MV4;
  MV4 = MV5;
  MV5 = MV6;
  MV6 = MV7;
  MV7 = MV8;
  MV8 = MV9;
  MV9 = MV10;
  
  return val;  
}

float TensionLipo::Tension_Lipo5 (){
  static float MV1 = 0.0;
  static float MV2 = 0.0;
  float MV3;
  float val;
  
  MV3 =  mesure(A4)* COEF_LIPO5;
  val = (MV1 + MV2 + MV3) / 3; // filter (average of 3 samples)
  MV1 = MV2; // shift
  MV2 = MV3;
  
  return val;
}

float TensionLipo::Tension_Lipo6 (){
  static float MV1 = 0.0;
  static float MV2 = 0.0;
  float MV3;
  float val;
  
  MV3 =  mesure(A5)* COEF_LIPO6;
  val = (MV1 + MV2 + MV3) / 3; // filter (average of 3 samples)
  MV1 = MV2; // shift
  MV2 = MV3;
  
  return val;    
}

// Detection of the number of elements of the pack
uint8_t TensionLipo::Detect () {
  float val;
  
  uint8_t nb_Lipo = 2;  // Minimum: 2S default.
  
  // We begin by reading cans to initialize filters
  for (uint8_t index = 0; index < 10; index++)
  {  
    val = TensionLipo::Tension_Lipo1 (); // Measurement element 1
    val = TensionLipo::Tension_Lipo2 (); // Measurement element 2
    val = TensionLipo::Tension_Lipo3 (); // Measurement element 3
    val = TensionLipo::Tension_Lipo4 (); // Measurement element 4    
    val = TensionLipo::Tension_Lipo5 (); // Measurement element 5   
    val = TensionLipo::Tension_Lipo6 (); // Measurement element 6
  }  

  val = TensionLipo::Tension_Lipo3 (); // Measurement element 3
  if (val > 8.0) {         // > 8V  
      nb_Lipo = 3;}
      
  val = TensionLipo::Tension_Lipo4 (); // Measurement element 4
  if (val > 11.0) {         // > 11V  
      nb_Lipo = 4;}  

  val = TensionLipo::Tension_Lipo5 (); // Measurement element 5
  if (val > 14.0) {         // > 14V  
      nb_Lipo = 5;}  

  val = TensionLipo::Tension_Lipo6 (); // Measurement element 6
  if (val > 17.0) {         // > 17V  
      nb_Lipo = 6;}        
      
  return nb_Lipo;  
}  

// Search the weakest element
float TensionLipo::Lipo_Mini (uint8_t nb_Lipo,float L1,float L2,float L3,float L4,float L5,float L6){
  float mini = 0.0;
  
  mini = L1;
  if (L2 < mini) {
	mini = L2;
  }

  if ((nb_Lipo >= 3) && (L3 < mini)){
    mini = L3;
  }

  if ((nb_Lipo >= 4) && (L4 < mini)){
    mini = L4;
  }
  
  if ((nb_Lipo >= 5) && (L5 < mini)){
    mini = L5;
  }  
  
  if ((nb_Lipo == 6) && (L6 < mini)){
    mini = L6;
  }  
  
  return mini;
}


// Voltage measurement (CAN)
uint16_t TensionLipo::mesure(uint8_t pin){
  //analogRead(pin);
  //delay(25);
  uint16_t val = analogRead(pin); 
  return val;
}



