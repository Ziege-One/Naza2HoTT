/*
   Naza2HoTT
   Ziege-One
   v1.0
 
 Arduino pro Mini 5v/16mHz w/ Atmega 328
 
 */

#define HOTTV4_RXTX 3           // Pin for HoTT telemetrie output
#define LEDPIN_OFF              PORTB &= ~(1<<5);
#define LEDPIN_ON               PORTB |= (1<<5);

// Module define
#define EAM_on                             // EAM Modul
#define GPS_on                             // GPS Modul
#define Vario_on                           // Vario Modul

#include "Message.h"
#include "Sensor.h"
#include <EEPROM.h>
#include "NazaDecoderLib.h"
#include <SoftwareSerial.h>

Sensor lipo;
 
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
struct HOTT_EAM_MSG     *hott_eam_msg = (struct HOTT_EAM_MSG *)&_hott_serial_buffer[0];
struct HOTT_GPS_MSG  *hott_gps_msg = (struct HOTT_GPS_MSG *)&_hott_serial_buffer[0];
struct HOTT_VARIO_MSG  *hott_vario_msg = (struct HOTT_VARIO_MSG *)&_hott_serial_buffer[0];
struct HOTT_TEXTMODE_MSG  *hott_txt_msg = (struct HOTT_TEXTMODE_MSG *)&_hott_serial_buffer[0];

// Alarm
int alarm_on_off_batt1 = 1; // 0=FALSE/Disable 1=TRUE/Enable   // Enable alarm by default for safety
char* alarm_on_off[13];      // For Radio OSD

static uint16_t alarm_min_volt = 900; // Min volt for alarm in mV
static uint16_t alarm_max_used = 1800; // Max Current Use for alarm in mA

// Timer for alarm
// for refresh time
int alarm_interval = 15000; // in ms
static unsigned long lastTime=0;  // in ms
unsigned long time=millis();      // in ms

// Messbereiche im Eprom
int Volt_Offset;
int Volt_COEF;
int Current_Offset;
int Current_COEF;

int GMessage::getVoltOffset() {return Volt_Offset; }
int GMessage::getVoltCOEF() {return Volt_COEF; }
int GMessage::getCurrentOffset() {return Current_Offset; }
int GMessage::getCurrentCOEF() {return Current_COEF; }

// For communication
static uint8_t octet1 = 0;  // reception
static uint8_t octet2 = 0;  // reception

// For saving settings in EEPROM
/*
 !WARNING!
 Writing takes 3.3ms.
 Maximum life of the EEPROM is 100000 writings/readings.
 Be careful not to use it too much, it is not replacable!
 */
#define adr_eprom_test 0                 // For the test for 1st time init of the Arduino (First power on)
#define adr_eprom_alarm_min_volt 2       // Default alarm min is 9.00v
#define adr_eprom_alarm_max_used 4       // Default alarm max is 1800mA
#define adr_eprom_alarm_on_off_batt1 6   // 0=FALSE/Disable 1=TRUE/Enable
#define adr_eprom_alarm_interval 8       // Audio warning alarm interval 
// Messbereiche im Eprom
#define adr_eprom_volt_offset 10         // Volt Offset in digi 
#define adr_eprom_volt_coef 12           // Volt COEF in x/10 mV
#define adr_eprom_current_offset 14      // Current Offset in digi 
#define adr_eprom_current_coef 16        // Current COEF in x/10 mA 

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

uint16_t gps_alt_m=9999;
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
int set_home_loop = 20000; //20000 about 4 seconds wait to set Homepiont

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
    write_eprom(adr_eprom_alarm_min_volt,alarm_min_volt);
    write_eprom(adr_eprom_alarm_max_used,alarm_max_used);
    write_eprom(adr_eprom_alarm_interval,alarm_interval);
    write_eprom(adr_eprom_alarm_on_off_batt1,alarm_on_off_batt1);
    // Messbereiche im Eprom
    write_eprom(adr_eprom_volt_offset,OffsetVolt);
    write_eprom(adr_eprom_volt_coef,COEF_Volt);
    write_eprom(adr_eprom_current_offset,OffsetCurrent);
    write_eprom(adr_eprom_current_coef,COEF_Current);
  }
  // Read saved values from EEPROM
    // alarm min on battery
    alarm_min_volt = read_eprom(adr_eprom_alarm_min_volt); // default is 9.00v if not change
    // alarm max on used mA
    alarm_max_used = read_eprom(adr_eprom_alarm_max_used); // default is 1800mA if not change
    // Timer for alarm
    alarm_interval = read_eprom(adr_eprom_alarm_interval); // default is 15000 ms if not change
    // Enable / Disable alarm bip
    alarm_on_off_batt1 = read_eprom(adr_eprom_alarm_on_off_batt1); // 0=FALSE/Disable 1=TRUE/Enable
    // Messbereiche im Eprom
    Volt_Offset = read_eprom(adr_eprom_volt_offset);
    Volt_COEF = read_eprom(adr_eprom_volt_coef);
    Current_Offset = read_eprom(adr_eprom_current_offset);
    Current_COEF = read_eprom(adr_eprom_current_coef);
    
    
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

void GMessage::init_eam_msg(){
  //puts to all Zero, then modifies the constants
  memset(hott_eam_msg, 0, sizeof(struct HOTT_EAM_MSG));   
  hott_eam_msg->start_byte = 0x7c;
  hott_eam_msg->eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
  hott_eam_msg->sensor_id = 0xe0;
  hott_eam_msg->stop_byte = 0x7d;
}

void GMessage::init_gps_msg(){
  //puts to all Zero, then modifies the constants
  memset(hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));   
  hott_gps_msg->startByte = 0x7c;
  hott_gps_msg->sensorID = HOTT_TELEMETRY_GPS_SENSOR_ID;
  hott_gps_msg->sensorTextID = 0xA0;
  hott_gps_msg->endByte = 0x7d;
}

void GMessage::init_vario_msg(){
  //met tous à Zero, puis on modifie les constantes
  memset(hott_vario_msg, 0, sizeof(struct HOTT_VARIO_MSG));   
  hott_vario_msg->startByte = 0x7c;
  hott_vario_msg->sensorID = HOTT_TELEMETRY_VARIO_SENSOR_ID;
  hott_vario_msg->sensorTextID = 0x90;
  hott_vario_msg->endByte = 0x7d;
}

// Sending the frame
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

  lipo.ReadSensor(); //Spannung und Strom Werte einlesen
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
    if (set_home_loop == 0)
      {
      is_set_home = 1;
      }
    set_home_loop--;  
  }
  
  if(SERIAL_HOTT.available() >= 2) {
    uint8_t octet1 = SERIAL_HOTT.read();
    switch (octet1) {
    case HOTT_BINARY_MODE_REQUEST_ID:
      { 
        uint8_t  octet2 = SERIAL_HOTT.read();
        
        // Demande RX Module =	$80 $XX
        switch (octet2) {
        
          #ifdef GPS_on
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
                    //hott_gps_msg->warning_beeps = 0x08; // no fix! so beep and voice alarm~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
                    hott_gps_msg->climbrate1s = 30000 + (gps_climb_speed * 100);
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
                    hott_gps_msg->climbrate1s = 30000 + (gps_climb_speed * 100);
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
          #endif
         
        #ifdef EAM_on
        case HOTT_TELEMETRY_EAM_SENSOR_ID: //0x8E
          {  
		  LEDPIN_ON

                  // init structure
		  init_eam_msg();

                  hott_eam_msg->cell_L[1] = 0 ; 			          //#7 Volt Cell_L 1 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_L[2] = 0 ;			            //#8 Volt Cell_L 2 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_L[3] = 0 ;			            //#9 Volt Cell_L 3 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_L[4] = 0 ;			            //#10 Volt Cell_L 4 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_L[5] = 0 ;			            //#11 Volt Cell_L 5 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_L[6] = 0 ;			            //#12 Volt Cell_L 6 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_L[7] = 0 ;                 //#13 Volt Cell_L 7 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[1] = 0 ;			            //#14 Volt Cell_L 1 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[2] = 0 ;			            //#15 Volt Cell_H 2 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[3] = 0 ;			            //#16 Volt Cell_H 3 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[4] = 0 ;			            //#17 Volt Cell_H 4 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[5] = 0 ;			            //#18 Volt Cell_H 5 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[6] = 0 ;			            //#19 Volt Cell_H 6 (in 2 mV increments, 210 == 4.20 V)
                  hott_eam_msg->cell_H[7] = 0 ;                             //#20 Volt Cell_H 7 (in 2 mV increments, 210 == 4.20 V)   
                  hott_eam_msg->Battery1 = (lipo.getVolt ()) * 10 ;         //#21 LSB battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                  hott_eam_msg->Battery2 = (lipo.getVCC ()) * 10 ;          //#23 LSB battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V only pos. voltages
                  hott_eam_msg->temperature1 = (lipo.getTemp()) + 20 ;      //#25 Temperature 1. Offset of 20. a value of 20 = 0°C
                  hott_eam_msg->temperature2 = 20 ;                         //#26 Temperature 2. Offset of 20. a value of 20 = 0°C
                  hott_eam_msg->altitude = gps_alt_m - home_altitude + 500 ;//#27 altitude in meters. offset of 500, 500 = 0m
                  hott_eam_msg->current = (lipo.getCurrent()) * 10 ;        //#29 current in 0.1A steps 100 == 10,0A
                  hott_eam_msg->main_voltage = (lipo.getVolt ()) * 10 ;     //#31 LSB Main power voltage using 0.1V steps 100 == 10,0V
                  hott_eam_msg->batt_cap =  (lipo.getBattCap()) / 10 ;      //#33 LSB used battery capacity in 10mAh steps
                  hott_eam_msg->climbrate_L = 30000 ;                       //#35 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
                  hott_eam_msg->climbrate3s = 120 ;                         //#37 climb rate in m/3sec. Value of 120 = 0m/3sec
                  hott_eam_msg->rpm = 0 ;                                   //#38 RPM in 10 RPM steps. 300 = 3000rpm
                  hott_eam_msg->Minutes = 0;                                //#40 Electric.Minutes (Time does start, when motor current is > 3 A)
                  hott_eam_msg->Seconds = 0;      	                        //#41 Electric.Seconds (1 byte)
                  hott_eam_msg->speed = gps_speed;                          //#42 LSB (air?) speed in km/h(?) we are using ground speed here per default
				 
  
		  send(sizeof(struct HOTT_EAM_MSG));
		  LEDPIN_OFF
		  break;
          }
          //end case EAM*/
         #endif
        
        #ifdef Vario_on
                case HOTT_TELEMETRY_VARIO_SENSOR_ID: //0x89
          {  
        LEDPIN_ON
        init_vario_msg();

                  hott_vario_msg->altitude = gps_alt_m - home_altitude + 500 ;;      //#6 LSB 244 244 = Low Byte Actual Altitude -2m (244+256=0m)
                  hott_vario_msg->maxAltitude;     //#8 LSB 244 244 = Low Byte Maximum Altitude. -2m (244+256=0m)
                  hott_vario_msg->minAltitude;           //#10 LSB 244 244 = Low Byte min. altitude -2m (244+256=0m)
                  hott_vario_msg->m1s = 30000 ;       //#12 LSB 48 Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s)
                  hott_vario_msg->m3s = 120 ;       //#14 LSB 48 Low Byte m/3s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/3s)
                  hott_vario_msg->m10s = 120 ;           //#16 LSB 48 Low Byte m/10s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/10s)
                  //hott_vario_msg->text[24] = "hallo";     //#18: 0 ASCII [1]
                  memset(hott_vario_msg->text,0x20,24);
                  snprintf((char*)hott_vario_msg->text,24, "Naza2HoTT by Ziege-One");
        
       
        send(sizeof(struct HOTT_VARIO_MSG));
        LEDPIN_OFF
        



        
        break;
          } //end case Vario*/
          #endif
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
          #ifdef EAM_on
          if (id_sensor == (HOTT_TELEMETRY_EAM_SENSOR_ID & 0x0f)) 
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
                    ligne_select = max(3,ligne_select-1); // never gets above line 3 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;
                    
                    else if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        page_settings-=1;
                        if (page_settings <1)    // unter Seite 1 dann Seite 2
                          page_settings = 2;
                        }
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        page_settings+=1;
                        if (page_settings >2)   // Über Seite 2 dann Seite 1
                          page_settings = 1;
                      }

                    //LINE 3 SELECTED = text[3]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 3 )
                      {
                        alarm_on_off_batt1 = 1;
                        *alarm_on_off = " Alarm :  ON";
                      }
                      
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 3 )
                      {
                        alarm_on_off_batt1 = 0;
                        *alarm_on_off = " Alarm : OFF";
                       }
                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 3)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_alarm_on_off_batt1,alarm_on_off_batt1);
                       }
                    
                    
                    //LINE 4 SELECTED = text[4]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 4 )
                      alarm_min_volt+=5;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 4 )
                      alarm_min_volt-=5;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 4 )
                      alarm_min_volt+=50;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 4 )
                      alarm_min_volt-=50;                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 4)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_alarm_min_volt,alarm_min_volt);
                       }

                    else if (alarm_min_volt>1820) // not over 18.2v
                        {
                          alarm_min_volt=5;
                        } 
                    else if (alarm_min_volt<1)  // not behind 0v
                        {
                       alarm_min_volt=420;
                        }

                    //LINE 5 SELECTED = text[5]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 5 )
                      alarm_max_used+=100;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 5 )
                      alarm_max_used-=100;
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 5)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_alarm_max_used,alarm_max_used);
                       }
                    else if (alarm_max_used>60000)
                    {
                       alarm_max_used=3000;  
                    }
                    else if (alarm_max_used<0)
                       {alarm_max_used=0; }

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
                    snprintf((char *)&hott_txt_msg->text[0],21," Alarms           <>");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Volt: %i.%iv",(int) lipo.getVolt (), ((int) (lipo.getVolt ()*100)) % 100);
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"Change settings :");
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,*alarm_on_off);
                    //line 4:
                    if (((int) (alarm_min_volt % 100)) == 5 || ((int) (alarm_min_volt % 100)) == 0)// Display management to put a 0 after the comma and the X.05v X.00v values ​​because the modulo return 0 or 5 for typical values ​​100,105,200,205,300,305, etc ...
                    snprintf((char *)&hott_txt_msg->text[4],21," Alarm Volt : %i.0%iv",(int) (alarm_min_volt/100),(int) (alarm_min_volt % 100)); // adding a 0
                    else // normal display
                    snprintf((char *)&hott_txt_msg->text[4],21," Alarm Volt : %i.%iv",(int) (alarm_min_volt/100),(int) (alarm_min_volt % 100)); // no need of adding 0
                    //line 5:
                    snprintf((char *)&hott_txt_msg->text[5],21," Used mA: %imA",(alarm_max_used));
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21," Alarm repeat: %is",(alarm_interval/1000));
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Strom2HoTT  %d/2",page_settings); //Showing page number running down the screen to the right
                    
                    hott_txt_msg->text[ligne_select][0] = '>';
                    _hott_invert_ligne(ligne_edit);
                    break;                    
                    }//END PAGE 1
                    
               case 2: // PAGE 2
                    {
                      // config test for the screen display has
                    if (id_key == HOTT_KEY_LEFT && ligne_edit == -1)
                        {
                        page_settings-=1;
                        if (page_settings <1)    // unter Seite 1 dann Seite 2
                          page_settings = 2;
                        }
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        page_settings+=1;
                        if (page_settings >2)   // Über Seite 2 dann Seite 1
                          page_settings = 1;
                      }
                                                      
                    else if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(3,ligne_select-1); // never gets above line 3 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    ligne_edit =  ligne_select ;

                    //LINE 3 SELECTED = text[3]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 3 )
                      Volt_Offset+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 3 )
                      Volt_Offset-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 3 )
                      Volt_Offset+=10;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 3 )
                      Volt_Offset-=10;                      
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 3)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_volt_offset,Volt_Offset);
                       }

                    else if (Volt_Offset>2000) // not over 2000
                        {
                          Volt_Offset=2000;
                        } 
                    else if (Volt_Offset<0)  // not behind 0
                        {
                       Volt_Offset=0;
                        }
                    
                    //LINE 4 SELECTED = text[4]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 4 )
                      Volt_COEF+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 4 )
                      Volt_COEF-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 4 )
                      Volt_COEF+=10;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 4 )
                      Volt_COEF-=10;                         
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 4)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_volt_coef,Volt_COEF);
                       }

                    else if (Volt_COEF>2000) // not over 2000
                        {
                          Volt_COEF=2000;
                        } 
                    else if (Volt_COEF<1)  // not behind 0
                        {
                       Volt_COEF=1;
                        }
                    
                    //LINE 5 SELECTED = text[5]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 5 )
                      Current_Offset+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 5 )
                      Current_Offset-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 5 )
                      Current_Offset+=10;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 5 )
                      Current_Offset-=10;                         
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 5)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_current_offset,Current_Offset);
                       }

                    else if (Current_Offset>2000) // not over 2000
                        {
                          Current_Offset=2000;
                        } 
                    else if (Current_Offset<0)  // not behind 0
                        {
                       Current_Offset=0;
                        }
                    
                    //LINE 6 SELECTED = text[6]
                    else if (id_key == HOTT_KEY_UP && ligne_select == 6 )
                      Current_COEF+=1;
                    else if (id_key == HOTT_KEY_DOWN && ligne_select == 6 )
                      Current_COEF-=1;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_select == 6 )
                      Current_COEF+=10;
                    else if (id_key == HOTT_KEY_LEFT && ligne_select == 6 )
                      Current_COEF-=10;                        
                    else if (id_key == HOTT_KEY_SET && ligne_edit == 6)
                      {
                       ligne_edit = -1 ;
                       write_eprom(adr_eprom_current_coef,Current_COEF);
                       }

                    else if (Current_COEF>2000) // not over 2000
                        {
                          Current_COEF=2000;
                        } 
                    else if (Current_COEF<1)  // not behind 0
                        {
                       Current_COEF=1000;
                        }                    
                    
                    // Showing page 2 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21," LIPO CAL         <");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Volt: %i.%iv",(int) lipo.getVolt (), ((int) (lipo.getVolt ()*100)) % 100);
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"Current: %i.%iA",(int) lipo.getCurrent(), ((int) (lipo.getCurrent()*100)) % 100);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21," Volt Off: %idigi",(Volt_Offset));
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21," Volt COEF: %i/10mv",(Volt_COEF)); 
                    //line 5:
                    snprintf((char *)&hott_txt_msg->text[5],21," Cur. Off: %idigi",(Current_Offset));
                    //line 6:
                    snprintf((char *)&hott_txt_msg->text[6],21," Cur. COEF: %i/10mA",(Current_COEF));
                    //line 7:
                    snprintf((char *)&hott_txt_msg->text[7],21,"Strom2HoTT  %d/2",page_settings); //Showing page number running down the screen to the right
                    int Volt_Offset;

                    hott_txt_msg->text[ligne_select][0] = '>';
                    _hott_invert_ligne(ligne_edit);
                    break;
                    }//END PAGE 2
                    
                  default://PAGE 
                  {
                    break;
                  }
                  
                  
                  
            }//END SETTINGS

          } // END IF
          #endif
          //********************************************************************************************************************
          #ifdef GPS_on
          if(id_sensor == (HOTT_TELEMETRY_GPS_SENSOR_ID & 0x0f)) {
            
            switch (page_settings) { //SETTINGS
              
              case 1://PAGE 1 SETTINGS
                    {
                    if (id_key == HOTT_KEY_UP && ligne_edit == -1)
                    ligne_select = min(6,ligne_select+1); // never gets above line 6 max
                    else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
                    ligne_select = max(4,ligne_select-1); // never gets above line 4 min
                    else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
                    //ligne_edit =  ligne_select ;
                    is_set_home = 0;
                    else if (id_key == HOTT_KEY_RIGHT && ligne_edit == -1)
                      {
                        if (page_settings >=4)// change it if you want more pages
                          page_settings = 1;
                        else
                          page_settings+=1;
                      }

                    // Showing page 1
                    //line 0:
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS NAZA / %s", is_set_home ? "Home set" : "Home ?");
                    //line 1:
                    snprintf((char *)&hott_txt_msg->text[1],21,"Current Position:");
                    //line 2:
                    snprintf((char *)&hott_txt_msg->text[2],21,"N %id %i' %i.%i\"",lat_D,lat_M,lat_S,lat_SS);
                    //line 3:
                    snprintf((char *)&hott_txt_msg->text[3],21,"E %id %i' %i.%i\"",lon_D,lon_M,lon_S,lon_SS);
                    //line 4:
                    snprintf((char *)&hott_txt_msg->text[4],21,"Start Position: %s", is_set_home ? "OK" : "NO");
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
                    //ligne_edit =  ligne_select ;
                    is_set_home = 0;
                    // Showing page 2 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS NAZA / %s", is_set_home ? "Home set" : "Home ?");
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
                    //ligne_edit =  ligne_select ;
                    is_set_home = 0;
                    // Showing page 3 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS NAZA / %s", is_set_home ? "Home set" : "Home ?");
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
                    //ligne_edit =  ligne_select ;
                    is_set_home = 0;
                    // Showing page 4 settings
                    //line 0:                                  
                    snprintf((char *)&hott_txt_msg->text[0],21,"GPS NAZA / %s", is_set_home ? "Home set" : "Home ?");
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
          #endif
          #ifdef GAM_on
          if(id_sensor == (HOTT_TELEMETRY_GAM_SENSOR_ID & 0x0f)) {
            snprintf((char *)&hott_txt_msg->text[0],21,"EAM sensor module    <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
          #endif
          #ifdef Vario_on
          if(id_sensor == (HOTT_TELEMETRY_VARIO_SENSOR_ID & 0x0f)) {         
            snprintf((char *)&hott_txt_msg->text[0],21,"VARIO sensor module  <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
          #endif
          /*else {
            snprintf((char *)&hott_txt_msg->text[0],21,"Unknow sensor module <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }*/
        }
        _hott_send_text_msg();
        //LEDPIN_OFF
        break;

      }
    }	
  }
}


//////////////////////////////////////////////////

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
          gps_heading_d = NazaDecoder.getHeadingNc();      // Get Heading (COMPASS) in degrees
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
    Serial.println("-----------Naza2HoTT-V3.0------------");
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
    Serial.print("Volt:"); Serial.print(lipo.getVolt(), 2);Serial.print("V = digit:");Serial.println(lipo.getVoltDigi());
    Serial.print("Current:"); Serial.print(lipo.getCurrent(), 2);Serial.print("A = digit:");Serial.println(lipo.getCurrentDigi());
    Serial.print("BattCap:"); Serial.print(lipo.getBattCap(), 0);Serial.println("mA");
    Serial.print("VCC:"); Serial.print(lipo.getVCC(), 2);Serial.println("V");
    Serial.print("Temp:"); Serial.print(lipo.getTemp(), 2);Serial.println("*C");
}
