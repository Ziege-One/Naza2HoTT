/*
   LIPOMETER v1.0
   http://johnlenfr.1s.fr
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328
 
 */

#define HOTTV4_RXTX 3           // Pin for HoTT telemetrie output
#define LEDPIN_OFF              PORTB &= ~(1<<5);
#define LEDPIN_ON               PORTB |= (1<<5);

#include "Message.h"
#include "Tension_Lipo.h"
#include <EEPROM.h>
#include "NazaDecoderLib.h"
#include <SoftwareSerial.h>

TensionLipo tension;
 
SoftwareSerial SERIAL_HOTT(HOTTV4_RXTX , HOTTV4_RXTX); // RX, TX

/**
 * Enables RX and disables TX
 */
static inline void hottV4EnableReceiverMode() {
  DDRD &= ~(1 << HOTTV4_RXTX);
  PORTD |= (1 << HOTTV4_RXTX);
}

/**
 * Enabels TX and disables RX
 */
static inline void hottV4EnableTransmitterMode() {
  DDRD |= (1 << HOTTV4_RXTX);
}

static uint8_t _hott_serial_buffer[173];   //creating a buffer variable to store the struct

// pointer to the buffer structures "_hott_serial_buffer"
struct HOTT_GAM_MSG     *hott_gam_msg = (struct HOTT_GAM_MSG *)&_hott_serial_buffer[0];
struct HOTT_TEXTMODE_MSG	*hott_txt_msg =	(struct HOTT_TEXTMODE_MSG *)&_hott_serial_buffer[0];
struct HOTT_GPS_MSG  *hott_gps_msg = (struct HOTT_GPS_MSG *)&_hott_serial_buffer[0];
struct HOTT_VARIO_MSG  *hott_vario_msg = (struct HOTT_VARIO_MSG *)&_hott_serial_buffer[0];


// Alarm
int alarm_on_off_batt1 = 1; // 0=FALSE/Disable 1=TRUE/Enable   // Enable alarm by default for safety
char* alarm_on_off[13];      // For Radio OSD
char* model_config[15];      // For Radio OSD
static uint16_t alarm_min1 = 360; // Min volt for alarm in mV
// Timer for alarm
// for refresh time
int alarm_interval = 15000; // in ms
static unsigned long lastTime=0;  // in ms
unsigned long time=millis();      // in ms

// For communication
static uint8_t octet1 = 0;  // reception
static uint8_t octet2 = 0;  // reception

// For Lipo
static uint8_t nb_Lipo = 4; // Number of items Lipo battery default before detection
float  Lipo_total = 0;      // Total voltage measured
float  lipo1 = 0.0;         // Voltage on each element
float  lipo2 = 0.0;
float  lipo3 = 0.0;
float  lipo4 = 0.0; 
float  lipo5 = 0.0; 
float  lipo6 = 0.0; 
float  lipo_mini_bat1 = 0.0; // Minimum voltage value element
uint8_t Jauge = 0;           // fuel gauge


// For saving settings in EEPROM
/*
 !WARNING!
 Writing takes 3.3ms.
 Maximum life of the EEPROM is 100000 writings/readings.
 Be careful not to use it too much, it is not replacable!
 */
#define adr_eprom_test 0                 // For the test for 1st time init of the Arduino (First power on)
#define adr_eprom_alarm_min1 2           // Default alarm min is 3.60v
#define adr_eprom_nb_cells_batt1 4       // Automaticaly detected and calculated
#define adr_eprom_alarm_on_off_batt1 6   // 0=FALSE/Disable 1=TRUE/Enable
#define adr_eprom_alarm_interval 8       // Audio warning alarm interval 

/*GPS variables*/
int gps_fix=-1;
float lat=0;
uint8_t lat_D=0;        // latitude Degree
uint16_t lat_M=0;       // latitude Minutes
uint16_t lat_S=0;       // latitude Seconds
uint16_t lat_SS=0;      // latitude SSeconds
uint8_t lat_home_D=0;   // latitude Degree
uint16_t lat_home_M=0;  // latitude Minutes
uint16_t lat_home_S=0;  // latitude Seconds
uint16_t lat_home_SS=0; // latitude Seconds
uint16_t lat_hott_M=0;  // latitude Minutes for Hott Graupner
uint32_t lat_hott_S=0;  // latitude Seconds for Hott Graupner

float lon=0;
uint8_t lon_D=0;
uint16_t lon_M=0;
uint16_t lon_S=0;
uint16_t lon_SS=0;
uint8_t lon_home_D=0;
uint16_t lon_home_M=0;
uint16_t lon_home_S=0;
uint16_t lon_home_SS=0;
uint16_t lon_hott_M=0;
uint32_t lon_hott_S=0;

uint8_t gps_alt_m=9999;
short gps_hdop_cm=0;
short gps_vdop_cm=0;
float gps_speed=0;
float gps_speed_avg=0;
float gps_speed_max=0;
float gps_heading_d=90.0;
double gps_cog=0;
double gps_climb_speed=0;
uint8_t gps_numsats=0;
uint8_t gps_year=00;
uint8_t gps_month=00;
uint8_t gps_day=00;
uint8_t gps_hour=00;
uint8_t gps_minute=00;
uint8_t gps_second=00;
uint16_t altitude_table[11];
float home_lat;
float home_lon;
int16_t home_altitude;
int16_t gps_alt_min = 9999;
int16_t gps_alt_max = 0000;
uint32_t gps_dist = 0000;
uint32_t gps_dist_max = 0000;

bool is_set_home = 0;

GMessage::GMessage(){

}
void GMessage::init(){
  
  SERIAL_HOTT.begin(SERIAL_COM_SPEED); // 19400 FOR GRAUPNER HOTT using SoftSerial lib.
  hottV4EnableReceiverMode(); 
  
  // Test for 1st time init of the Arduino (First power on)
  int test = read_eprom(adr_eprom_test);
  if (test != 123)
  {
    write_eprom(adr_eprom_test,123);
    write_eprom(adr_eprom_alarm_min1,alarm_min1);
    write_eprom(adr_eprom_alarm_on_off_batt1,alarm_on_off_batt1);
  }
  // Read saved values from EEPROM
    // alarm min on battery
    alarm_min1 = read_eprom(adr_eprom_alarm_min1); // default is 3.60v if not change
    // Enable / Disable alarm bip
    alarm_on_off_batt1 = read_eprom(adr_eprom_alarm_on_off_batt1); // 0=FALSE/Disable 1=TRUE/Enable
    alarm_interval = read_eprom(adr_eprom_alarm_interval);
    tension.begin ();
    nb_Lipo = tension.Detect (); // detection du nombre d'elements 2S a 6S
    
  // init altitude table GPS/Vario
  for (int i=0;i < 10;i++) altitude_table[i] = 0;
}

uint16_t GMessage::read_eprom(int address){
  return  (uint16_t) EEPROM.read(address) * 256 + EEPROM.read(address+1) ;
}

void GMessage::write_eprom(int address,uint16_t val){
  EEPROM.write(address, val  / 256);
  EEPROM.write(address+1,val % 256 );
}


void GMessage::init_gam_msg(){
  //puts to all Zero, then modifies the constants
  memset(hott_gam_msg, 0, sizeof(struct HOTT_GAM_MSG));   
  hott_gam_msg->start_byte = 0x7c;
  hott_gam_msg->gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
  hott_gam_msg->sensor_id = 0xd0;
  hott_gam_msg->stop_byte = 0x7d;
}

void GMessage::init_gps_msg(){
  //puts to all Zero, then modifies the constants
  memset(hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));   
  hott_gps_msg->startByte = 0x7c;
  hott_gps_msg->sensorID = HOTT_TELEMETRY_GPS_SENSOR_ID;
  hott_gps_msg->sensorTextID = 0xA0;
  hott_gps_msg->endByte = 0x7d;
}
/*
void GMessage::init_vario_msg(){
  //met tous à Zero, puis on modifie les constantes
  memset(hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));   
  hott_vario_msg->startByte = 0x7c;
  hott_vario_msg->sensorID = HOTT_TELEMETRY_VARIO_SENSOR_ID;
  hott_vario_msg->sensorTextID = 0x90;
  hott_vario_msg->endByte = 0x7d;
}*/

// Emission de la trame
void GMessage::send(int lenght){ 
  uint8_t sum = 0;
  hottV4EnableTransmitterMode(); 
  delay(5);
  for(int i = 0; i < lenght-1; i++){
    sum = sum + _hott_serial_buffer[i];
    SERIAL_HOTT.write (_hott_serial_buffer[i]);
    delayMicroseconds(HOTTV4_TX_DELAY);
  }  
  //Emision checksum
  SERIAL_HOTT.write (sum);
  delayMicroseconds(HOTTV4_TX_DELAY);

  hottV4EnableReceiverMode();
}

void GMessage::main_loop(){ 
  
  // STARTING MAIN PROGRAM
  static byte page_settings = 1; // page number to display settings
  
  decode_gps_naza();             // record infos from the GPS
  update_table_altitude();       // update altitudes every 1s
  
  if( is_set_home==0 && gps_fix>=3 && gps_numsats>=6){ // FIX 3D mode or DGPS  (we need at least 6 satellites to calculate an accurate home position)
    home_lat = lat;
    home_lon = lon;
    lat_home_D = lat_D;
    lat_home_M = lat_M;
    lat_home_S = lat_S;
    lat_home_SS = lat_SS;
    lon_home_D = lon_D;
    lon_home_M = lon_M;
    lon_home_S = lon_S;
    lon_home_SS = lon_SS;
    home_altitude = gps_alt_m;
    is_set_home = 1;
  }
  
  if(SERIAL_HOTT.available() >= 2) {
    uint8_t octet1 = SERIAL_HOTT.read();
    switch (octet1) {
    case HOTT_BINARY_MODE_REQUEST_ID:
      { 
        uint8_t  octet2 = SERIAL_HOTT.read();
        
        // Demande RX Module =	$80 $XX
        switch (octet2) {
        
          case HOTT_TELEMETRY_GPS_SENSOR_ID: //0x8A
          {  
          //LEDPIN_ON
          init_gps_msg();
            
            switch (gps_fix){
              
              // Naza gives: NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4
              // Hott gives: nofix = '0x2d' 2D = '0x32' 3D = '0x33'  Dgps: '0x44'  

                  case 0 : // NO_FIX = '0x2d'
                    // HOTT DATA:
                    hott_gps_msg->GPSFixChar = 0x2d;     // Displays a ' ' to show nothing or clear the old value
                    hott_gps_msg->GPS_fix = 0x2d;        // Displays a ' ' to show nothing or clear the old value
                    hott_gps_msg->altitude = 500 ;       // <=> 0     
                    hott_gps_msg->climbrate1s = 30000;   // <=> 0 
                    hott_gps_msg->climbrate3s = 120;     // <=> 0 
                    hott_gps_msg->GPSNumSat = gps_numsats ;
                    // RADIO SCREEN:
                    hott_gps_msg->alarmInverse1 = 1; // invers DIST. OSD
                    hott_gps_msg->alarmInverse2 = 1; // invers COORD OSD
                    // RADIO VOICE ALARM
                    hott_gps_msg->warning_beeps = 0x08; // no fix! so beep and voice alarm
                    break;
                    
                  case 2 : // FIX_2D = '0x32'
                    // HOTT DATA:
                    hott_gps_msg->flightDirection = gps_cog / 2;
                    hott_gps_msg->GPSSpeed = 0;                     // waiting for satellites
                    hott_gps_msg->LatitudeNS = 0;                   // O=North, 1= South
                    hott_gps_msg->LatitudeMin = lat_hott_M;
                    hott_gps_msg->LatitudeSec =  lat_hott_S;
                    hott_gps_msg->longitudeEW = 0;                  //east = 0, west = 1
                    hott_gps_msg->longitudeMin = lon_hott_M;
                    hott_gps_msg->longitudeSec = lon_hott_S;
                    hott_gps_msg->distance = 0;                     // waiting for satellites
                    hott_gps_msg->altitude = 500;                   // <=> 0  offset 500 Graupner
                    hott_gps_msg->climbrate1s = 30000;              // <=> 0  waiting for satellites
                    hott_gps_msg->climbrate3s = 120;                // <=> 0  waiting for satellites
                    hott_gps_msg->GPSNumSat = gps_numsats ;         // waiting for satellites
                    hott_gps_msg->GPS_fix =  0x32;                  // Dgps: '0x44' 2D = '0x32' 3D = '0x33' nofix = '0x2d'
                    hott_gps_msg->GPSFixChar =  0x32;
                    hott_gps_msg->gps_time_h = gps_hour;
                    hott_gps_msg->gps_time_m = gps_minute;
                    hott_gps_msg->gps_time_s = gps_second;
                    hott_gps_msg->msl_altitude = 0;                 // waiting for satellites
                    // RADIO SCREEN:
                    hott_gps_msg->alarmInverse1 = 1; // invers DIST. OSD
                    hott_gps_msg->alarmInverse2 = 0; // non invers COORD OSD
                    // RADIO VOICE ALARM
                    hott_gps_msg->warning_beeps = 0x00; // No Z but X and Y are ok
                    break;
                    
                  case 3 : // FIX_3D = '0x33'
                    // HOTT DATA:
                    hott_gps_msg->flightDirection = gps_cog / 2;
                    hott_gps_msg->GPSSpeed = gps_speed;
                    hott_gps_msg->LatitudeNS = 0;                    // O=North, 1= South
                    hott_gps_msg->LatitudeMin = lat_hott_M;
                    hott_gps_msg->LatitudeSec =  lat_hott_S;
                    hott_gps_msg->longitudeEW = 0;                   //east = 0, west = 1
                    hott_gps_msg->longitudeMin = lon_hott_M;
                    hott_gps_msg->longitudeSec = lon_hott_S;
                    hott_gps_msg->distance = gps_dist; 
                    hott_gps_msg->altitude = gps_alt_m - home_altitude + 500; // offset 500 Graupner
                    //hott_gps_msg->climbrate1s = 30000  + (altitude_table[0] - altitude_table[1])*100 ;
                    hott_gps_msg->climbrate1s = 30000 + gps_climb_speed;
                    hott_gps_msg->climbrate3s = 120  + (altitude_table[0] - altitude_table[3]) ;
                    hott_gps_msg->GPSNumSat = gps_numsats ; 
                    hott_gps_msg->GPS_fix =  0x33;                   // Dgps: '0x44' 2D = '0x32' 3D = '0x33' nofix = '0x2d'
                    hott_gps_msg->GPSFixChar =  0x33;
                    hott_gps_msg->HomeDirection = calculateAngle (); 
                    //hott_gps_msg->angleXdirection                  // not implemented yet
                    //hott_gps_msg->angleYdirection                  // not implemented yet
                    //hott_gps_msg->angleZdirection                  // not implemented yet
                    hott_gps_msg->gps_time_h = gps_hour;
                    hott_gps_msg->gps_time_m = gps_minute;
                    hott_gps_msg->gps_time_s = gps_second;
                    hott_gps_msg->msl_altitude = gps_alt_m;
                    // RADIO SCREEN:
                    hott_gps_msg->alarmInverse1 = 0; // non invers DIST. OSD
                    hott_gps_msg->alarmInverse2 = 0; // non invers COORD OSD
                    // RADIO VOICE ALARM
                    hott_gps_msg->warning_beeps = 0x00; // no beep or warning, everything is OK
                    break;
                    
                 case 4 : // FIX_DGPS = '0x44'
                    // HOTT DATA:
                    hott_gps_msg->flightDirection = gps_cog / 2;
                    hott_gps_msg->GPSSpeed = gps_speed;
                    hott_gps_msg->LatitudeNS = 0; // O=North, 1= South
                    hott_gps_msg->LatitudeMin = lat_hott_M;
                    hott_gps_msg->LatitudeSec =  lat_hott_S;
                    hott_gps_msg->longitudeEW = 0; //east = 0, west = 1
                    hott_gps_msg->longitudeMin = lon_hott_M;
                    hott_gps_msg->longitudeSec = lon_hott_S;
                    hott_gps_msg->distance = gps_dist; 
                    hott_gps_msg->altitude = gps_alt_m - home_altitude + 500; // offset 500 Graupner
                    //hott_gps_msg->climbrate1s = 30000  + (altitude_table[0] - altitude_table[1])*100 ;
                    hott_gps_msg->climbrate1s = 30000 + gps_climb_speed;
                    hott_gps_msg->climbrate3s = 120  + (altitude_table[0] - altitude_table[3]) ;
                    hott_gps_msg->GPSNumSat = gps_numsats ; 
                    hott_gps_msg->GPS_fix =  0x44; // Dgps: '0x44' 2D = '0x32' 3D = '0x33' nofix = '0x2d'
                    hott_gps_msg->GPSFixChar =  0x44;
                    hott_gps_msg->HomeDirection = (int) calculateAngle ();     
                    //hott_gps_msg->angleXdirection
                    //hott_gps_msg->angleYdirection
                    //hott_gps_msg->angleZdirection
                    hott_gps_msg->gps_time_h = gps_hour;
                    hott_gps_msg->gps_time_m = gps_minute;
                    hott_gps_msg->gps_time_s = gps_second;
                    hott_gps_msg->msl_altitude = gps_alt_m;
                    // RADIO SCREEN:
                    hott_gps_msg->alarmInverse1 = 0; // non invers DIS OSD
                    hott_gps_msg->alarmInverse2 = 0; // non invers coord OSD
                    // RADIO VOICE ALARM
                    hott_gps_msg->warning_beeps = 0x00; // no beep or warning, everything is OK
                    break;
                         
                    default : // NO_FIX = '0x2d'
                    // HOTT DATA:
                    hott_gps_msg->GPSFixChar = 0x2d;     // Displays a ' ' to show nothing or clear the old value
                    hott_gps_msg->GPS_fix = 0x2d;        // Displays a ' ' to show nothing or clear the old value
                    hott_gps_msg->altitude = 500 ;       // <=> 0     
                    hott_gps_msg->climbrate1s = 30000;   // <=> 0 
                    hott_gps_msg->climbrate3s = 120;     // <=> 0 
                    hott_gps_msg->GPSNumSat = gps_numsats ;
                    // RADIO SCREEN:
                    hott_gps_msg->alarmInverse1 = 1; // invers DIST. OSD
                    hott_gps_msg->alarmInverse2 = 1; // invers COORD OSD
                    // RADIO VOICE ALARM
                    hott_gps_msg->warning_beeps = 0x08; // no fix! so beep and voice alarm
                    break;
            }
            
            send(sizeof(struct HOTT_GPS_MSG));
          
          //LEDPIN_OFF  
              break;
          }
          
        /*case HOTT_TELEMETRY_VARIO_SENSOR_ID: //0x89
          {  
			  LEDPIN_ON
			  init_vario_msg();
			  
			 
			  send(sizeof(struct HOTT_VARIO_MSG));
			  LEDPIN_OFF
		  break;
          }
         */
         
        /*case HOTT_TELEMETRY_GEA_SENSOR_ID: //0x89
          {  
			  LEDPIN_ON
			  //init_gea_msg();
			  
			  
			  //send(sizeof(struct HOTT_GEA_MSG));
			  LEDPIN_OFF
		  break;
          }
         */
        
        case HOTT_TELEMETRY_GAM_SENSOR_ID: //0x8D
          {    
               LEDPIN_ON
            
            // init structure
               init_gam_msg();
            
            // Set values to 0 for clean screen
               setTemp (0,2);                       // not use
               setTemp (0,1);                       // not use
               setClimbrate_M3(120);                // not use
               setVoltage2(0);                      // not use
              
            // Use values of the GPS
               setAltitudeInMeters (gps_alt_m);     // use gps infos
               setClimbrate(30000+gps_climb_speed); // use gps infos
               setSpeed(gps_speed);                 // use gps infos

            // Voltage Measurement
             lipo1 = tension.Tension_Lipo1();
             lipo2 = tension.Tension_Lipo2();
             lipo3 = tension.Tension_Lipo3();
             lipo4 = tension.Tension_Lipo4();
             lipo5 = tension.Tension_Lipo5();
             lipo6 = tension.Tension_Lipo6();

            // Calculate the value of each Elements
              switch (nb_Lipo){
                case 2 : // 2S Lipo 
                Lipo_total = lipo2;
                lipo2 = lipo2 - lipo1;
                break;  
            
                case 3 :// 3S Lipo 
                Lipo_total = lipo3;
                lipo3 = lipo3 - lipo2;
                lipo2 = lipo2 - lipo1;       
                setLipo (lipo3,3);   
                break;        
            
                case 4 :// 4S Lipo 
                Lipo_total = lipo4;
                lipo4 = lipo4 - lipo3;
                lipo3 = lipo3 - lipo2;
                lipo2 = lipo2 - lipo1;
                setLipo (lipo3,3);      
                setLipo (lipo4,4);  
                break;  
              
                case 5 :// 5S Lipo 
                Lipo_total = lipo5;
                lipo5 = lipo5 - lipo4;
                lipo4 = lipo4 - lipo3;
                lipo3 = lipo3 - lipo2;
                lipo2 = lipo2 - lipo1;
                setLipo (lipo3,3);    
                setLipo (lipo4,4);      
                setLipo (lipo5,5);  
                break;          
              
                case 6 :// 6S Lipo 
                Lipo_total = lipo6;
                lipo6 = lipo6 - lipo5;
                lipo5 = lipo5 - lipo4;
                lipo4 = lipo4 - lipo3;
                lipo3 = lipo3 - lipo2;
                lipo2 = lipo2 - lipo1;
                setLipo (lipo3,3);    
                setLipo (lipo4,4);      
                setLipo (lipo5,5);  
                setLipo (lipo6,6);  
                break;         
              
                default : ;
              }     
              
              // Set 1S and 2S Lipo by default      
              setLipo (lipo1,1);
              setLipo (lipo2,2);
              
              // set lipo voltages on Battery 1 and Main voltage
              setVoltage1(Lipo_total);    // Batt 1
              setMainVoltage(Lipo_total); // Main

               // Searching Minus
               // Search the weakest element
               // to gauge display
               // radio and alarm management
               lipo_mini_bat1 = tension.Lipo_Mini(nb_Lipo,lipo1,lipo2,lipo3,lipo4,lipo5,lipo6);
               setVoltage2(lipo_mini_bat1);    // Batt 2
               
               // calculating tonnage linearly
               // with Math function: map(value, fromLow, fromHigh, toLow, toHigh)
                //
                //- 4,20V 100%
                //- 4,10V 90%
                //- 3,97V 80%
                //- 3.92V 70%	 
                //- 3,87V 60%
                //- 3,83V 50% :Voltage to store your battery until the heyday ...
                //- 3,79V 40%
                //- 3,75V 30%
                //- 3,70V 20% : it is time to load! 
                //- 3,60V 10%
                //- 3,30V 5%
                //- 3,00V 0% 
                
              // 100%
              if (lipo_mini_bat1 > 4.15)
              {
                Jauge = 100;
              }
              // 90 to 100%
              //4,20V 100%
              //4,10V 90%
              if (lipo_mini_bat1 > 4.1 && lipo_mini_bat1 <= 4.15)
              {
                Jauge = map(lipo_mini_bat1,4.1,4.2,90,100);
              }
              // 80 to 90%
              //4,10V 90%
              //3,97V 80%
               if (lipo_mini_bat1 > 3.97 && lipo_mini_bat1 <= 4.1)
              {
                Jauge = map(lipo_mini_bat1,3.97,4.1,80,90);
              }
              // 70 to 80%
              //3,97V 80%
              //3.92V 70%
              if (lipo_mini_bat1 > 3.92 && lipo_mini_bat1 <= 3.97)
              {
                Jauge = map(lipo_mini_bat1,3.92,3.97,70,80);
              }
              // 60 to 70%
              //3.92V 70%	 
              //3,87V 60%
              if (lipo_mini_bat1 > 3.87 && lipo_mini_bat1 <= 3.92)
              {
                Jauge = map(lipo_mini_bat1,3.87,3.92,60,70);
              }
              // 50 to 60%
              //3,87V 60%
              //3,83V 50%
              if (lipo_mini_bat1 > 3.83 && lipo_mini_bat1 <= 3.87)
              {
                Jauge = map(lipo_mini_bat1,3.83,3.87,50,60);
              }
              // 40 to 50%
              //3,83V 50%
              //3,79V 40%
              if (lipo_mini_bat1 > 3.79 && lipo_mini_bat1 <= 3.83)
              {
                Jauge = map(lipo_mini_bat1,3.79,3.83,40,60);
              }
              // 30 to 40%
              //3,79V 40%
              //3,75V 30%
              if (lipo_mini_bat1 > 3.75 && lipo_mini_bat1 <= 3.79)
              {
                Jauge = map(lipo_mini_bat1,3.75,3.79,30,40);
              }
              // 20 to 30%
              //3,75V 30%
              //3,70V 20% 
              if (lipo_mini_bat1 > 3.7 && lipo_mini_bat1 <= 3.75)
              {
                Jauge = map(lipo_mini_bat1,3.70,3.75,20,30);
              }
              // 10 to 20%
              //3,70V 20%
              //3,60V 10%
              if (lipo_mini_bat1 > 3.6 && lipo_mini_bat1 <= 3.7)
              {
                Jauge = map(lipo_mini_bat1,3.6,3.7,10,20);
              }
              // 5 to 10%
              //3,60V 10%
              //3,30V 5%
              if (lipo_mini_bat1 > 3.3 && lipo_mini_bat1 <= 3.6)
              {
                Jauge = map(lipo_mini_bat1,3.3,3.6,5,10);
              }
              // 0 to 5%
              //3,30V 5%
              //3,00V 0% 
              if (lipo_mini_bat1 > 3 && lipo_mini_bat1 <= 3.3)
              {
                Jauge = map(lipo_mini_bat1,3,3.3,0,5);
              }
              // 0%
              if (lipo_mini_bat1 > 0 && lipo_mini_bat1 <= 3)
              {
                Jauge = 0;
              }
                
                
               time=millis();
               if (time-lastTime>alarm_interval)  // if at least alarm_interval in ms have passed
               {
                     // Check for alarm beep
                     if (((lipo_mini_bat1*100) < alarm_min1) && alarm_on_off_batt1 == 1)
                     {
                     hott_txt_msg->warning_beeps =  ALARME_TENSION_SEUIL    ; // alarm beep or voice
                     }
                 lastTime=time;  // reset timer
               }
            
               // Fuel gauge display
               setFuelPercent (Jauge);
               // View percentage of lipo voltage behind fuel in place of milliliters
               setFuelMilliliters(Jauge);
            
            // sending all data
            send(sizeof(struct HOTT_GAM_MSG));
            break;
          } //end case GAM*/
        } //end case octet 2
        break;
      }

    case HOTT_TEXT_MODE_REQUEST_ID:
      {
        //LEDPIN_ON
        uint8_t  octet3 = SERIAL_HOTT.read();
        byte id_sensor = (octet3 >> 4);
        byte id_key = octet3 & 0x0f;
        static byte ligne_select = 4 ;
        static int8_t ligne_edit = -1 ;
        hott_txt_msg->start_byte = 0x7b;
        hott_txt_msg->esc = 0;
        hott_txt_msg->warning_beeps = 0;
        
        memset((char *)&hott_txt_msg->text, 0x20, HOTT_TEXTMODE_MSG_TEXT_LEN);
        hott_txt_msg->stop_byte = 0x7d;

        if (id_key == HOTT_KEY_LEFT && page_settings == 1)
        {   
          hott_txt_msg->esc = 0x01;
        }
        else
        {
          if (id_sensor == (HOTT_TELEMETRY_GAM_SENSOR_ID & 0x0f)) 
          {
            switch (page_settings) { //SETTINGS
              
              case 1://PAGE 1 SETTINGS
              
                    {
                    // test if alarm active for display ON or OFF the screen
                    if (alarm_on_off_batt1 == 0)
                      *alarm_on_off = " Alarm : OFF";
                      else
                      *alarm_on_off = " Alarm :  ON";
                                           
                    if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(4,ligne_select-1); // never gets above line 4 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        if (page_settings >=1)// change it if you want more pages
                          page_settings = 1;
                        else
                          page_settings+=1;
                      }
                      
                      
                      
                    //LINE 4 SELECTED = text[4]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 4 )
                      {
                        alarm_on_off_batt1 = 1;
                        *alarm_on_off = " Alarm :  ON";
                      }
                      
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 4 )
                      {
                        alarm_on_off_batt1 = 0;
                        *alarm_on_off = " Alarm : OFF";
                       }
                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 4)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_alarm_on_off_batt1,alarm_on_off_batt1);
                       }
                    
                    
                    //LINE 5 SELECTED = text[5]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 5 )
                      alarm_min1+=5;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 5 )
                      alarm_min1-=5;
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 5)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_alarm_min1,alarm_min1);
                       }

                    else if (alarm_min1>420) // not over 4.2v
                        {
                          alarm_min1=5;
                        } 
                    else if (alarm_min1<1)  // not behind 0v
                        {
                       alarm_min1=420;
                        }

                    //LINE 6 SELECTED = text[6]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 6 )
                      alarm_interval+=1000;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 6 )
                      alarm_interval-=1000;
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 6)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_alarm_interval,alarm_interval);
                       }
                    else if (alarm_interval>60000)
                    {
                       alarm_interval=1000;  
                    }
                    else if (alarm_interval<0)
                       {alarm_interval=0; }
                              
                    // Showing page 1
                    
                    //line 0:
                    snprintf((char *)&hott_txt_msg->text[0],21," LIPO    <");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Lipo Total: %i.%iv",(int) Lipo_total, ((int) (Lipo_total*100)) % 100);
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"Cells : %iS",nb_Lipo);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,"Change settings :");
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21,*alarm_on_off);
                    //line 5:
                    if (((int) (alarm_min1 % 100)) == 5 || ((int) (alarm_min1 % 100)) == 0)// Display management to put a 0 after the comma and the X.05v X.00v values ​​because the modulo return 0 or 5 for typical values ​​100,105,200,205,300,305, etc ...
                    snprintf((char *)&hott_txt_msg->text[5],21," Alarm value : %i.0%iv",(int) (alarm_min1/100),(int) (alarm_min1 % 100)); // adding a 0
                    else // normal display
                    snprintf((char *)&hott_txt_msg->text[5],21," Alarm value : %i.%iv",(int) (alarm_min1/100),(int) (alarm_min1 % 100)); // no need of adding 0
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21," Alarm repeat: %is",(alarm_interval/1000));
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Naza2HoTT  %d/1",page_settings); //Showing page number running down the screen to the right
                    
                    hott_txt_msg->text[ligne_select][0] = '>';
                    _hott_invert_ligne(ligne_edit);
                    break;                    
                    }//END PAGE 1
                  
                  default://PAGE 
                  {
                    break;
                  }
                  
                  
                  
            }//END SETTINGS

          } // END IF
          
          //********************************************************************************************************************
          else if(id_sensor == (HOTT_TELEMETRY_GPS_SENSOR_ID & 0x0f)) {
            
            switch (page_settings) { //SETTINGS
              
              case 1://PAGE 1 SETTINGS
                    {
                    if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(4,ligne_select-1); // never gets above line 4 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;
                    
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        if (page_settings >=4)// change it if you want more pages
                          page_settings = 1;
                        else
                          page_settings+=1;
                      }

                    // Showing page 1
                    //line 0:
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS  DJI  NAZA");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Current Position:");
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"N %id %i' %i.%i\"",lat_D,lat_M,lat_S,lat_SS);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,"E %id %i' %i.%i\"",lon_D,lon_M,lon_S,lon_SS);
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21,"Start Position:");
                    //line 5: 
                    snprintf((char *)&hott_txt_msg->text[5],21,"N %id %i' %i.%i\"",lat_home_D,lat_home_M,lat_home_S,lat_home_SS);
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21,"E %id %i' %i.%i\"",lon_home_D,lon_home_M,lon_home_S,lon_home_SS);
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Naza2HoTT  %d/4",page_settings); //Showing page number running down the screen to the right
                    
                    //hott_txt_msg->text[ligne_select][0] = '>';
                    //_hott_invert_ligne(ligne_edit);
                    break;                    
                    }//END PAGE 1
                  
                  case 2: // PAGE 2
                    {
                      // config test for the screen display has
                    if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        if (page_settings <=1)
                          page_settings = 4;
                        else
                          page_settings-=1;
                        }
                     else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        if (page_settings >=4)// change it if you want more pages
                          page_settings = 1;
                        else
                          page_settings+=1;
                      }
                                                      
                    else if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(4,ligne_select-1); // never gets above line 4 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;
                    
                    // Showing page 2 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS  DJI  NAZA");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Dist. Current: %im",(int) gps_dist);
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"Dist. Maximum: %im",(int) gps_dist_max);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,"COG: %i",(int) gps_cog);
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21,"Direction    : %i",(int) gps_heading_d);  
                    //line 5:
                    snprintf((char *)&hott_txt_msg->text[5],21,"Average Speed: %ikm/h",(int) gps_speed_avg);
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21,"Maximum Speed: %ikm/h",(int) gps_speed_max);
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Naza2HoTT  %d/4",page_settings); //Showing page number running down the screen to the right
                    
                    //hott_txt_msg->text[ligne_select][0] = '>';
                    //_hott_invert_ligne(ligne_edit);
                    break;
                    }//END PAGE 2
                    
                    
                    case 3: // PAGE 3
                    {
                      // config test for the screen display has
                    if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        if (page_settings <=1)
                          page_settings = 4;
                        else
                          page_settings-=1;
                        }
                        else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        if (page_settings >=4)// change it if you want more pages
                          page_settings = 1;
                        else
                          page_settings+=1;
                      }
                                                      
                    else if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(4,ligne_select-1); // never gets above line 4 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;
                    
                    // Showing page 3 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS  DJI  NAZA");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Altitude    : %i",gps_alt_m);
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"Alt. Min.   : %i",gps_alt_min);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,"Alt. Max.   : %i",gps_alt_max);
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21,"Alt. Ground : %i",(int) (gps_alt_m - home_altitude));
                    //line 5:
                    snprintf((char *)&hott_txt_msg->text[5],21,"Diff./Sec   : %i",(altitude_table[0] - altitude_table[1])*100);
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21,"Diff./3Sec  : %i",(altitude_table[0] - altitude_table[3]));
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Naza2HoTT  %d/4",page_settings); //Showing page number running down the screen to the right
                    
                    //hott_txt_msg->text[ligne_select][0] = '>';
                    //_hott_invert_ligne(ligne_edit);
                    break;
                    }//END PAGE 3
                    
                    case 4: // PAGE 4
                    {
                      // config test for the screen display has
                    if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        if (page_settings <=1)
                          page_settings = 4;
                        else
                          page_settings-=1;
                        }
                        
                        else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        if (page_settings >=4)// change it if you want more pages
                          page_settings = 1;
                        else
                          page_settings+=1;
                      }
                                                      
                    else if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(4,ligne_select-1); // never gets above line 4 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;
                    
                    // Showing page 4 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS  DJI  NAZA");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"%i.%i.20%i  %i:%i:%i",gps_day,gps_month,gps_year,gps_hour,gps_minute,gps_second);
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"Fix : %i",gps_fix);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,"Sat.: %i",gps_numsats);
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21,"VDOP: %icm",gps_hdop_cm);
                    //line 5:
                    snprintf((char *)&hott_txt_msg->text[5],21,"HDOP: %icm",gps_vdop_cm);
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21,"COG: %i",(int) gps_cog);
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Naza2HoTT %d/4",page_settings); //Showing page number running down the screen to the right
                    
                    //hott_txt_msg->text[ligne_select][0] = '>';
                    //_hott_invert_ligne(ligne_edit);
                    break;
                    }//END PAGE 4
                    
            }//END SETTINGS
            
            
            
            //snprintf((char *)&hott_txt_msg->text[5],21," test1 '%d' ",octet3);
            //snprintf((char *)&hott_txt_msg->text[6],21,"s'%d'  e'%d'",ligne_select,ligne_edit);            
            
             
            
          }

          else if(id_sensor == (HOTT_TELEMETRY_GEA_SENSOR_ID & 0x0f)) {
            snprintf((char *)&hott_txt_msg->text[0],21,"EAM sensor module    <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
          else if(id_sensor == (HOTT_TELEMETRY_VARIO_SENSOR_ID & 0x0f)) {         
            snprintf((char *)&hott_txt_msg->text[0],21,"VARIO sensor module  <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
          else {
            snprintf((char *)&hott_txt_msg->text[0],21,"Unknow sensor module <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
        }
        _hott_send_text_msg();
        //LEDPIN_OFF
        break;

      }
    }	
  }
}


//////////////////////////////////////////////////
// Main voltage (0.1V resolution)
void GMessage::setMainVoltage(float tension){
  hott_gam_msg->main_voltage = (uint16_t) (tension * 10);
}

// battery 1 (0.1V resolution)
void GMessage::setVoltage1(float volt){
  hott_gam_msg->Battery1 =  (uint16_t) (volt * 10); 
}

// battery 2 (0.1V resolution)
void GMessage::setVoltage2(float volt){
  hott_gam_msg->Battery2 = (uint16_t) (volt * 10); 
}


// Element n Lipo (0.2V resolution)
void GMessage::setLipo(float volt, int lipo){

  if (lipo >= 1 and lipo <= 6)
  {
    lipo--;
    hott_gam_msg->cell[lipo] =  (uint16_t) 100 * volt / 2 ; 
    
    if (volt/2 <= hott_gam_msg->min_cell_volt || hott_gam_msg->min_cell_volt ==0)
    {
      hott_gam_msg->min_cell_volt = (uint16_t) 100 * volt/2 ;
      hott_gam_msg->min_cell_volt_num = lipo;
    }
  }
}


// Relative altitude in meters : -500 / +9999 [m]
void GMessage::setAltitudeInMeters(uint16_t alti){
  hott_gam_msg->altitude = alti + 500;
}

// temperature : -20 a +235° C 
// 
void GMessage::setTemp(int temp, int capteur){
  if(temp < -20)
    temp = -20;
  else if(temp > 234)
    temp = 235;
  if (capteur < 1)
    capteur = 1;
  if (capteur > 2)
    capteur = 2; 
  if (capteur == 1)
    hott_gam_msg->temperature1 = temp + 20;
  else if (capteur == 2)
    hott_gam_msg->temperature2 = temp + 20;
}

// tank level : 0%, 25%, 50%, 75%, 100%  
// 
void GMessage::setFuelPercent(uint8_t pourcent){
  if(pourcent > 100)
    pourcent = 100;
  else if(pourcent < 0)
    pourcent = 0;
  hott_gam_msg->fuel_procent = pourcent ;
}

// tank level ml : 0 a 65535 ml
// 
void GMessage::setFuelMilliliters(uint16_t ml){
  hott_gam_msg->fuel_ml = ml ;
}

// rotation : 0 a 655350 tr/min
// 

void GMessage::setRPM(uint16_t rpm){
  hott_gam_msg->rpm = rpm ;
} 

// Set climbrates
void GMessage::setClimbrate(uint16_t climb_L){
  hott_gam_msg->climbrate_L =  climb_L;
}
void GMessage::setClimbrate_M3(int climb3s){
  hott_gam_msg->climbrate3s =  climb3s;
}

// Set speed
void GMessage::setSpeed(float speed){
  hott_gam_msg->speed =  (int) speed;
}


//
// Audible alarm on radio
//
void GMessage::alarme (uint8_t son){
  hott_gam_msg->warning_beeps = son;
}



void GMessage::_hott_send_text_msg() {
  for(byte *_hott_msg_ptr = hott_txt_msg->text[0]; _hott_msg_ptr < &hott_txt_msg->stop_byte ; _hott_msg_ptr++){
    if (*_hott_msg_ptr == 0)
      *_hott_msg_ptr = 0x20;
  }  
  //_hott_send_msg(_hott_serial_buffer, sizeof(struct HOTT_TEXTMODE_MSG));
  send(sizeof(struct HOTT_TEXTMODE_MSG));
}


char * GMessage::_hott_invert_all_chars(char *str) {
  return _hott_invert_chars(str, 0);
}


char * GMessage::_hott_invert_chars(char *str, int cnt) {
  if(str == 0) return str;
  int len = strlen(str);
  if((len < cnt)  && cnt > 0) len = cnt;
  for(int i=0; i< len; i++) {
    str[i] = (byte)(0x80 + (byte)str[i]);
  }
  return str;
}


void GMessage::_hott_invert_ligne(int ligne) {
  if (ligne>= 0 && ligne<= 7)
    for(int i=0; i< 21; i++) {
      if (hott_txt_msg->text[ligne][i] == 0)   //reversing the null character (end of string)
        hott_txt_msg->text[ligne][i] = (byte)(0x80 + 0x20);
      else
        hott_txt_msg->text[ligne][i] = (byte)(0x80 + (byte)hott_txt_msg->text[ligne][i]);
    }
}

void GMessage::decode_gps_naza() {

    if(Serial.available() > 0)
    {
      
    uint8_t decodedMessage = NazaDecoder.decode(Serial.read()); 
    
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
        
        LEDPIN_ON // to view that serial communication at 115200bauds is OK
        
        lat =        NazaDecoder.getLat();                                 // XX.XXXXXXX DEG
        lat_D =      (int) lat;                                            // True DEG
        lat_M =      (int) ((lat-lat_D)*60);                               // True MIN
        lat_S  =     (int) ((((lat-lat_D)*60)-lat_M)*60);                  // True SEC
        lat_SS  =    (int) ((((((lat-lat_D)*60)-lat_M)*60)- lat_S)*10000); // True SEC after dot
        lat_hott_M = (int) (lat_D*100)+ lat_M;                             // Hott MIN
        lat_hott_S = ((lat * 100000) - (lat_D * 100000)) * 6;              // Hott SEC
        lat_hott_S = lat_hott_S % 10000;                                   // Hott SEC
        
        
        
        lon =        NazaDecoder.getLon();                                 // XX.XXXXXXX DEG
        lon_D =      (int) lon;                                            // True DEG
        lon_M =      (int) ((lon-lon_D)*60);                               // True MIN
        lon_S  =     (int) ((((lon-lon_D)*60)-lon_M)*60);                  // True SEC
        lon_SS  =    (int) (((((lon-lon_D)*60)-lon_M)*60-lon_S)*10000);    // True SEC after dot
        lon_hott_M = (int) (lon_D*100) + lon_M;                            // Hott MIN
        lon_hott_S = ((lon * 100000) - (lon_D * 100000)) * 6;              // Hott SEC
        lon_hott_S = lon_hott_S % 10000;                                   // Hott SEC
        
        gps_alt_m =  NazaDecoder.getGpsAlt();            // Get Altitude in meters

        gps_fix =    NazaDecoder.getFixType();           // get fix type : NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4
        gps_numsats= NazaDecoder.getNumSat();            // Get satellites
        gps_speed =  3.6 * (NazaDecoder.getSpeed());     // Get speed in km/h nasa is m/s
        gps_year =   NazaDecoder.getYear();              // Get GPS Year
        gps_month =  NazaDecoder.getMonth();             // Get GPS Month
        gps_day =    NazaDecoder.getDay();               // Get GPS Day
        gps_hour =   NazaDecoder.getHour();              // Get GPS Hour
        gps_minute = NazaDecoder.getMinute();            // Get GPS Minute
        gps_second = NazaDecoder.getSecond();            // Get GPS Second
        
        gps_climb_speed = NazaDecoder.getGpsVsi();       // clim speed in m/s
        
        gps_hdop_cm = NazaDecoder.getHdop();             // Get HDop
        gps_vdop_cm = NazaDecoder.getVdop();             // Get VDop
        gps_cog     = NazaDecoder.getCog();              // Get COG
        
        if( is_set_home==1 )                             // Calculate distance if home point is set
        gps_dist    = calculateDistance();
        break;
        
      case NAZA_MESSAGE_COMPASS:
          gps_heading_d = 360 - (NazaDecoder.getHeadingNc());      // Get Heading (COMPASS) in degrees
        break;
    }
  }
}

uint32_t GMessage::seconds() {
  return millis() / 1000;
}

void GMessage::update_table_altitude()
{
  static uint32_t table_now = 0;
  uint32_t now = seconds();

  if ((now - table_now) >= 1)
  {
    table_now = now ;
    for (int i = 9 ;  i >= 0; i--) {
      altitude_table[i+1] = altitude_table[i];
    }
    // alt  max / min
    if( is_set_home==1 )                             // Calculate alt min max if home point is set
      if (gps_alt_m<gps_alt_min)
        gps_alt_min = gps_alt_m;
      if (gps_alt_m>gps_alt_max)
        gps_alt_max = gps_alt_m;

    altitude_table[0] = gps_alt_m ; // update table
    
    //update max speed at the same time
    if (gps_speed > gps_speed_max)
    gps_speed_max = gps_speed;
    
    //update max Distance at the same time
    if (gps_dist > gps_dist_max)
    gps_dist_max = gps_dist;
    
    //update average speed at the same time
    if (gps_speed > 0.9)
    gps_speed_avg = (gps_speed_avg + gps_speed) / 2;
    
  }
}

uint32_t GMessage::calculateDistance()
//(float lat1, float long1, float lat2, float long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers

  float long1 = lon;
  float long2 = home_lon;
  float lat1 = lat;
  float lat2 = home_lat;

  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

float GMessage::calculateAngle ()
//(float lat1, float long1, float lat2, float long2) 
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  
  float long1 = lon;
  float long2 = home_lon;
  float lat1 = lat;
  float lat2 = home_lat;
  
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

void GMessage::debug(){
   // FOR DEBUG
    Serial.println("------------------------");
    Serial.print("Alt:"); Serial.println(gps_alt_m, 7);
    Serial.print("Fix:"); Serial.println(gps_fix);
    Serial.print("Sat:");Serial.println(gps_numsats);
    Serial.print("heading:");Serial.println(gps_heading_d);
    Serial.print("COG:");Serial.println(gps_cog);
    Serial.print("20");Serial.print(gps_year);Serial.print("."); Serial.print(gps_month);Serial.print(".");Serial.println(gps_day);
    Serial.print(gps_hour); Serial.print("h");Serial.print(gps_minute);Serial.print("m");Serial.print(gps_second);Serial.println("s UTC");
    Serial.print("Lat naza:"); Serial.println(NazaDecoder.getLat(),7);
    Serial.print("Lon naza:"); Serial.println(NazaDecoder.getLon(),7);
    Serial.print("Lat:"); Serial.println(lat*100000);
    Serial.print("D:"); Serial.print(lat_D);
    Serial.print("M:"); Serial.print(lat_M);
    Serial.print("S:"); Serial.print(lat_S); Serial.print(".");Serial.println(lat_SS);
    Serial.print("Lon:"); Serial.println(lon*100000);
    Serial.print("D:"); Serial.print(lon_D);
    Serial.print("M:"); Serial.print(lon_M);
    Serial.print("S:"); Serial.print(lon_S); Serial.print(".");Serial.println(lon_SS);
    Serial.print("Lipo 1:"); Serial.print(tension.Tension_Lipo1());Serial.println("V");
    Serial.print("Lipo 2:"); Serial.print(tension.Tension_Lipo2());Serial.println("V");
    Serial.print("Lipo 3:"); Serial.print(tension.Tension_Lipo3());Serial.println("V");
    Serial.print("Lipo 4:"); Serial.print(tension.Tension_Lipo4());Serial.println("V");
}
