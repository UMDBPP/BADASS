// ***********************************
// Includes

// Include sensor libraries

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BNO055.h"
#include <Servo.h> 
#include <Adafruit_MCP9808.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1015.h>

// include logging libraries
#include <SPI.h>
#include <SD.h>

// include communication libraries
#include <XBee.h>
#include"CCSDS_Xbee/ccsds_xbee.h"
#include "BADASS_Interface.h"

extern uint32_t _SendCtr;
extern uint32_t _RcvdCtr;

// ***********************************
// Definitions

// hardware configuration
#define SD_PIN 53
#define ADS_CURRENT_PIN 2
#define ADS_VOLTAGE_PIN 3
#define ServoPIN        10

// constants
#define SEALEVELPRESSURE_HPA (1013.25)

// ***********************************
// Declarations

// declare the log file
File logFile;

// declare the servo motor
Servo servo1;

// declare the BNO
Adafruit_BNO055 bno = Adafruit_BNO055(0,0x29);
Adafruit_BNO055 bno2 = Adafruit_BNO055(1,0x28);

// declare the MCP9808 temperature sensor objects
Adafruit_MCP9808 tempsensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808();

// declare the BME temp/pressure/humidity sensor
Adafruit_BME280 bme;

// declare the ADC
Adafruit_ADS1015 ads(0x4A);

// this bitfield, and the masks above, define which values get output in
//  telemetry 

// define common telemetry combinations
#define TLM_HK 16761861 // [
#define TLM_ATT 901
#define TLM_INT 62927874

uint32_t tlmctrl = 16761861;
//uint32_t tlmctrl = 0b0001000000000111;


////// Sensors 
// defines bits in initstatus for sensor status
#define INITMASK_BNO    	0x0001
#define INITMASK_ADS 		0x0002
#define INITMASK_MCP9808_1 	0x0004
#define INITMASK_MCP9808_2 	0x0008
#define INITMASK_BME 		0x0010
#define INITMASK_SD 	    0x0020
#define INITMASK_xbee		0x0040
#define INITMASK_servo 		0x0080

////// Modes
#define MODE_Manual 	0x00
#define MODE_Auto  		0x01

// ***********************************
// Initalizations

// Initial parameters
//  These parameters control program execution and may be changed via 
//    commands during execution

// execution rate of the program
uint16_t desiredctrltime = 100; // [ms]
uint16_t desiredIMUlogtime = 100; // [ms]
uint16_t desiredHKlogtime = 100; // [ms]
uint16_t desiredcommtime = 1000; // [ms]

// command the servo
bool ServoEnableFlg = true;
bool El_Cmd_Polarity = true;

uint8_t op_mode = MODE_Auto;

// target in the NED frame
imu::Vector<3> target_ned = imu::Vector<3>(1.0, 0.0, 0.0);

// transform between imu and body frame
imu::Quaternion quat_imu2body = imu::Quaternion(0.7071, 0.0, 0.0, 0.7071);

// coordinate axes in the body frame
imu::Vector<3> x_axis = imu::Vector<3>( 1.0, 0.0, 0.0);
imu::Vector<3> y_axis = imu::Vector<3>( 0.0, 1.0, 0.0);
imu::Vector<3> z_axis = imu::Vector<3>( 0.0, 0.0, 1.0);

// xbee address to sent telemetry to
uint8_t tlm_addr = 0x02;

// actuator stops
float max_el_ang = 170;
float min_el_ang = 10;

// manual mode command angles
float man_el_ang = 0.0;

// Program Memory
// These values are calculated/generated during the exectuion of
//  the program

// timing
uint16_t ctrl_cycle_start = 0;
uint16_t HKlog_cycle_start = 0;
uint16_t IMUlog_cycle_start = 0;
uint16_t comm_cycle_start = 0;

uint16_t ctrl_cycle_time;
uint16_t HKlog_cycle_time;
uint16_t IMUlog_cycle_time;
uint16_t comm_cycle_time;

// data buffers
uint8_t incomingByte[100];
uint8_t telemetry_data[200];

uint8_t tlm_len = 0;
uint16_t tlm_seq_cnt = 0;
int pkt_type = 0;
uint16_t cmdrcvdcnt = 0;
uint8_t fcncode = 0;
int APID = 0;
int PktType = 0;
int bytesread = 0;

int16_t servo_cmd = 0;

// hk values
float temp1 = 0;
float temp2 = 0;
float tempbme = 0.0;
float pres = 0.0;
float alt = 0.0;
float humid = 0.0;
uint16_t raw_current, raw_voltage;

uint16_t initstatus = 0;

// make it long enough to hold your longest file name, plus a null terminator
char filename[16]; 

uint16_t xbee_addr = 01;
uint16_t xbee_PanID = 0x0B0B;

uint8_t sys_cal = 0, gyro_cal = 0, accel_cal = 0, mag_cal = 0;
uint8_t sys_cal2 = 0, gyro_cal2 = 0, accel_cal2 = 0, mag_cal2 = 0;

uint8_t sys_stat = 0, st_res = 0, sys_err = 0;
int i = 0;
float pterr_az = 0.0, pterr_el = 0.0;
float cmd_az_ang = 0.0, cmd_el_ang = 0.0;

sensors_event_t event; 

// target is defined in NED frame (North, East, Down)
imu::Vector<3> target_body;
imu::Vector<3> in_plane_tgt;
imu::Vector<3> euler;

imu::Quaternion quat_ned2body;
imu::Quaternion quat_ned2imu;
imu::Quaternion quat_ned2imu2;

// ***********************************
// Functions

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(" xxx");  
  Serial.println(F("------------------------------------"));
  Serial.println("");
  delay(500);
}
void displaySensorDetails2(void)
{
  sensor_t sensor;
  bno2.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(" xxx");  
  Serial.println(F("------------------------------------"));
  Serial.println("");
  delay(500);
}
static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void cart2spher(imu::Vector<3> vec, float &theta, float &phi){
  theta = atan2(vec(1),vec(0));
  phi = atan2(sqrt(pow(vec(0),2)+pow(vec(1),2)),vec(2));
}

void enforcestops(float &az, float &el){
  // enforce El between +/- 90
  if(el < -(float)PI/2){
    el = -(float)PI/2;
  }
  else if(el*2 > (float)PI){
    el = (float)PI/2;
  }
}

imu::Vector<3> normtoplane(imu::Vector<3> NormVec, imu::Vector<3> Tgt) {
    // calculate elevation angle
	// inplanetgt = cross(normal,cross(target,normal))
	// inplanetgt = inplanetgt ./norm(inplanetgt);
  return imu::Vector<3>( - Tgt(0)*(pow(NormVec(0),2) - 1) - NormVec(0)*NormVec(1)*Tgt(1) - NormVec(0)*NormVec(2)*Tgt(2),
    NormVec(1)*NormVec(2)*Tgt(2) - NormVec(0)*NormVec(1)*Tgt(0) - Tgt(1)*(pow(NormVec(1),2) - 1),
    - Tgt(2)*(pow(NormVec(2),2) - 1) - NormVec(0)*NormVec(2)*Tgt(0) - NormVec(1)*NormVec(2)*Tgt(1));
  
}
    
void calcPtErr(imu::Vector<3> target_body, imu::Vector<3> x_axis, imu::Vector<3> y_axis, float &pterr_az, float &pterr_el){
	
	// calculate az error
    pterr_az = atan2(target_body(1),target_body(0));

    // calculate el error
    in_plane_tgt = normtoplane(y_axis, target_body);
    in_plane_tgt.normalize();
    pterr_el = acos(in_plane_tgt.dot(x_axis));
    if(in_plane_tgt(2) < 0){
      pterr_el = - pterr_el;
    }
}


void logData(uint32_t tlmctrl){
// log requested telemetry to file on SD card
  
  // open the file
  logFile = SD.open(filename, FILE_WRITE);
  
  // telemetry compilation
  if(tlmctrl & TLMMask_TlmCtrl){

    logFile.print(tlmctrl);
    logFile.print(", ");

  }
  
  //logFile.print(cycle_start_time);
  //logFile.print(", ");
  
  if(tlmctrl & TLMMask_TargetNED){
    logFile.print(target_ned(0), 4);
    logFile.print(", ");
    logFile.print(target_ned(1), 4);
    logFile.print(", ");
    logFile.print(target_ned(2), 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_BNOCal){
    logFile.print(sys_cal);
    logFile.print(", ");
    logFile.print(gyro_cal);
    logFile.print(", ");
    logFile.print(accel_cal);
    logFile.print(", ");
    logFile.print(mag_cal);
    logFile.print(", ");
    logFile.print(sys_cal2);
    logFile.print(", ");
    logFile.print(gyro_cal2);
    logFile.print(", ");
    logFile.print(accel_cal2);
    logFile.print(", ");
    logFile.print(mag_cal2);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_EulerAng){
    logFile.print(euler.x(), 4);
    logFile.print(", ");
    logFile.print(euler.y(), 4);
    logFile.print(", ");
    logFile.print(euler.z(), 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_q_imu2body){
    logFile.print(quat_imu2body.x(), 4);
    logFile.print(", ");
    logFile.print(quat_imu2body.y(), 4);
    logFile.print(", ");
    logFile.print(quat_imu2body.z(), 4);
    logFile.print(", ");
    logFile.print(quat_imu2body.w(), 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_q_ned2imu){
    logFile.print(quat_ned2imu.x(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu.y(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu.z(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu.w(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu2.x(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu2.y(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu2.z(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2imu2.w(), 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_q_ned2body){
    logFile.print(quat_ned2body.x(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2body.y(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2body.z(), 4);
    logFile.print(", ");
    logFile.print(quat_ned2body.w(), 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_v_targetbody){
    logFile.print(target_body(0), 4);
    logFile.print(", ");
    logFile.print(target_body(1), 4);
    logFile.print(", ");
    logFile.print(target_body(2), 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_AzElErr){
    logFile.print(pterr_az, 4);
    logFile.print(", ");
    logFile.print(pterr_el, 4);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_ElCmd){
    logFile.print(servo_cmd);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_CycleTime){
    logFile.print(ctrl_cycle_start);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_CmdRcvd){
    logFile.print(fcncode);
    logFile.print(", ");  
  }
  
  if(tlmctrl & TLMMask_DesiredCycTime){
    logFile.print(desiredctrltime);
    logFile.print(", ");
  }
    
  if(tlmctrl & TLMMask_Temp1){
    logFile.print(temp1);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_Temp2){
    logFile.print(temp2);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_TempBME){
    logFile.print(tempbme);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_Pres){
    logFile.print(pres);
    logFile.print(", ");  
  }
  
  if(tlmctrl & TLMMask_PresAlt){
    logFile.print(alt);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_Humid){
    logFile.print(humid);
    logFile.print(", ");
  }
  
  if(tlmctrl & TlmMask_Current){
    logFile.print(raw_current);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_Volt){
    logFile.print(raw_voltage);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_InitStat){
    logFile.print(initstatus);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_MsgSent){
    logFile.print(_SendCtr);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_MsgRcvd){
    logFile.print(_RcvdCtr);
    logFile.print(", ");
  }
  
  if(tlmctrl & TLMMask_Mode){
    logFile.print(op_mode);
    logFile.print(", ");
  }
  
  // close the file
  logFile.println();
  logFile.close();
  
}
  
uint8_t compileTLM(uint32_t tlmctrl){
// compiles the requested telemetry into a bytearray

  // initalize position in array
  uint8_t tlm_pos = 0;
  
  // telemetry compilation
  if(tlmctrl & TLMMask_TlmCtrl){
    tlm_pos = addIntToTlm(tlmctrl, telemetry_data, tlm_pos);

    Serial.print(" tlmctrl: ");
    Serial.print(tlmctrl);

  }

  if(tlmctrl & TLMMask_TargetNED){
    tlm_pos = addFloatToTlm( target_ned(0), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_ned(1), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_ned(2), telemetry_data, tlm_pos);

    Serial.print(" target_ned: ");
    Serial.print(target_ned(0), 4);
    Serial.print(" ");
    Serial.print(target_ned(1), 4);
    Serial.print(" ");
    Serial.print(target_ned(2), 4);
  }

  if(tlmctrl & TLMMask_BNOCal){
    tlm_pos = addIntToTlm(sys_cal, telemetry_data, tlm_pos);
    tlm_pos = addIntToTlm(gyro_cal, telemetry_data, tlm_pos);
    tlm_pos = addIntToTlm(accel_cal, telemetry_data, tlm_pos);
    tlm_pos = addIntToTlm(mag_cal, telemetry_data, tlm_pos);

    Serial.print(" Cal: ");
    Serial.print(sys_cal);
    Serial.print(" ");
    Serial.print(gyro_cal);
    Serial.print(" ");
    Serial.print(accel_cal);
    Serial.print(" ");
    Serial.print(mag_cal);

  }

  if(tlmctrl & TLMMask_EulerAng){
    tlm_pos = addFloatToTlm( euler.x(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( euler.y(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( euler.z(), telemetry_data, tlm_pos);

    Serial.print(" euler ang: ");
    Serial.print("[");
    Serial.print(euler.x(), 4);
    Serial.print("; ");
    Serial.print(euler.y(), 4);
    Serial.print("; ");
    Serial.print(euler.z(), 4);
    Serial.print("]");

  }

  if(tlmctrl & TLMMask_q_imu2body){
    tlm_pos = addFloatToTlm( quat_imu2body.x(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_imu2body.y(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_imu2body.z(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_imu2body.w(), telemetry_data, tlm_pos);

    Serial.print(" imu2body: ");
    Serial.print("[");
    Serial.print(quat_imu2body.x(), 4);
    Serial.print("; ");
    Serial.print(quat_imu2body.y(), 4);
    Serial.print("; ");
    Serial.print(quat_imu2body.z(), 4);
    Serial.print("; ");
    Serial.print(quat_imu2body.w(), 4);
    Serial.print("]");
  }

  if(tlmctrl & TLMMask_q_ned2imu){
    tlm_pos = addFloatToTlm( quat_ned2imu.x(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_ned2imu.y(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_ned2imu.z(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_ned2imu.w(), telemetry_data, tlm_pos);

    Serial.print(" ned2imu: ");
    Serial.print("[");
    Serial.print(quat_ned2imu.x(), 4);
    Serial.print("; ");
    Serial.print(quat_ned2imu.y(), 4);
    Serial.print("; ");
    Serial.print(quat_ned2imu.z(), 4);
    Serial.print("; ");
    Serial.print(quat_ned2imu.w(), 4);
    Serial.print("]");    
  }

  if(tlmctrl & TLMMask_q_ned2body){
    tlm_pos = addFloatToTlm( quat_ned2body.x(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_ned2body.y(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_ned2body.z(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( quat_ned2body.w(), telemetry_data, tlm_pos);

    Serial.print(" ned2body: ");
    Serial.print("[");
    Serial.print(quat_ned2body.x(), 4);
    Serial.print("; ");
    Serial.print(quat_ned2body.y(), 4);
    Serial.print("; ");
    Serial.print(quat_ned2body.z(), 4);
    Serial.print("; ");
    Serial.print(quat_ned2body.w(), 4);
    Serial.print("]");

  }

  if(tlmctrl & TLMMask_v_targetbody){
    tlm_pos = addFloatToTlm( target_body(0), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_body(1), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_body(2), telemetry_data, tlm_pos);

    Serial.print(" target_body: ");
    Serial.print("[");
    Serial.print(target_body(0), 4);
    Serial.print("; ");
    Serial.print(target_body(1), 4);
    Serial.print("; ");
    Serial.print(target_body(2), 4);
    Serial.print("]");

  }

  if(tlmctrl & TLMMask_AzElErr){
    tlm_pos = addFloatToTlm( pterr_az, telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( pterr_el, telemetry_data, tlm_pos);

    Serial.print(" az_el err: ");
    Serial.print("[");
    Serial.print(pterr_az,4);
    Serial.print("; ");
    Serial.print(pterr_el,4);
    Serial.print("]");
  }

  if(tlmctrl & TLMMask_ElCmd){
    tlm_pos = addIntToTlm(servo_cmd, telemetry_data, tlm_pos);

    Serial.print(" Cmd: ");
    Serial.print(servo_cmd);
  }

  if(tlmctrl & TLMMask_CycleTime){
    tlm_pos = addIntToTlm(ctrl_cycle_start, telemetry_data, tlm_pos);

    Serial.print(" CycTime: ");
    Serial.print(ctrl_cycle_start);
  }
    
  if(tlmctrl & TLMMask_CmdRcvd){
    tlm_pos = addIntToTlm(fcncode, telemetry_data, tlm_pos);

    Serial.print(" FcnCodeEcho: ");
    Serial.print(fcncode);
  }
  
  if(tlmctrl & TLMMask_DesiredCycTime){
    tlm_pos = addIntToTlm(desiredctrltime, telemetry_data, tlm_pos);

    Serial.print(" DesiredCycTime: ");
    Serial.print(desiredctrltime);
  }

  if(tlmctrl & TLMMask_Temp1){
    tlm_pos = addFloatToTlm(temp1, telemetry_data, tlm_pos);

    Serial.print(" temp1: ");
    Serial.print(temp1);
  }

  if(tlmctrl & TLMMask_Temp2){
    tlm_pos = addFloatToTlm(temp2, telemetry_data, tlm_pos);

    Serial.print(" temp2: ");
    Serial.print(temp2);
  }

  if(tlmctrl & TLMMask_TempBME){
    tlm_pos = addFloatToTlm(tempbme, telemetry_data, tlm_pos);

    Serial.print(" tempbme: ");
    Serial.print(tempbme);
  }

  if(tlmctrl & TLMMask_Pres){
    tlm_pos = addFloatToTlm(pres, telemetry_data, tlm_pos);

    Serial.print(" pres: ");
    Serial.print(pres);
  }

  if(tlmctrl & TLMMask_PresAlt){
    tlm_pos = addFloatToTlm(alt, telemetry_data, tlm_pos);

    Serial.print(" alt: ");
    Serial.print(alt);
  }

  if(tlmctrl & TLMMask_Humid){
    tlm_pos = addFloatToTlm(humid, telemetry_data, tlm_pos);

    Serial.print(" humid: ");
    Serial.print(humid);
  }

  if(tlmctrl & TlmMask_Current){
    tlm_pos = addIntToTlm(raw_current, telemetry_data, tlm_pos);

    Serial.print(" current: ");
    Serial.print(raw_current);
  }

  if(tlmctrl & TLMMask_Volt){
    tlm_pos = addIntToTlm(raw_voltage, telemetry_data, tlm_pos);

    Serial.print(" voltage: ");
    Serial.print(raw_voltage);
  }
  
  if(tlmctrl & TLMMask_InitStat){
    tlm_pos = addIntToTlm(initstatus, telemetry_data, tlm_pos);

    Serial.print(" initstat: ");
    Serial.print(initstatus);
  }
  
  if(tlmctrl & TLMMask_MsgSent){
    tlm_pos = addIntToTlm(_SendCtr, telemetry_data, tlm_pos);

    Serial.print(" msg sent: ");
    Serial.print(_SendCtr);
  }

  if(tlmctrl & TLMMask_MsgRcvd){
    // counter maintained by ccsds_xbee.c
    tlm_pos = addIntToTlm(_RcvdCtr, telemetry_data, tlm_pos);

    Serial.print(" tlmcnt: ");
    Serial.print(_RcvdCtr);
  }

  if(tlmctrl & TLMMask_Mode){
    tlm_pos = addIntToTlm(op_mode, telemetry_data, tlm_pos);

    Serial.print(" mode: ");
    Serial.print(op_mode);
  }
  
  
  if(tlmctrl){
    Serial.println(" ");
  }
  
  return tlm_pos;
}

void BADASS(){
  Serial.println(F("BBBBBBBBBBBBBBBBB              AAA              DDDDDDDDDDDDD                 AAA                SSSSSSSSSSSSSSS   SSSSSSSSSSSSSSS"));
  Serial.println(F("B::::::::::::::::B            A:::A             D::::::::::::DDD             A:::A             SS:::::::::::::::SSS:::::::::::::::S"));
  Serial.println(F("B::::::BBBBBB:::::B          A:::::A            D:::::::::::::::DD          A:::::A           S:::::SSSSSS::::::S:::::SSSSSS::::::S"));
  Serial.println(F("BB:::::B     B:::::B        A:::::::A           DDD:::::DDDDD:::::D        A:::::::A          S:::::S     SSSSSSS:::::S     SSSSSSS"));
  Serial.println(F("B::::B     B:::::B        A:::::::::A            D:::::D    D:::::D      A:::::::::A         S:::::S           S:::::S            "));
  Serial.println(F("B::::B     B:::::B       A:::::A:::::A           D:::::D     D:::::D    A:::::A:::::A        S:::::S           S:::::S            "));
  Serial.println(F("B::::BBBBBB:::::B       A:::::A A:::::A          D:::::D     D:::::D   A:::::A A:::::A        S::::SSSS         S::::SSSS         "));
  Serial.println(F("B:::::::::::::BB       A:::::A   A:::::A         D:::::D     D:::::D  A:::::A   A:::::A        SS::::::SSSSS     SS::::::SSSSS    "));
  Serial.println(F("B::::BBBBBB:::::B     A:::::A     A:::::A        D:::::D     D:::::D A:::::A     A:::::A         SSS::::::::SS     SSS::::::::SS  "));
  Serial.println(F("B::::B     B:::::B   A:::::AAAAAAAAA:::::A       D:::::D     D:::::DA:::::AAAAAAAAA:::::A           SSSSSS::::S       SSSSSS::::S "));
  Serial.println(F("B::::B     B:::::B  A:::::::::::::::::::::A      D:::::D     D:::::A:::::::::::::::::::::A               S:::::S           S:::::S"));
  Serial.println(F("B::::B     B:::::B A:::::AAAAAAAAAAAAA:::::A     D:::::D    D:::::A:::::AAAAAAAAAAAAA:::::A              S:::::S           S:::::S"));
  Serial.println(F("BB:::::BBBBBB::::::A:::::A             A:::::A  DDD:::::DDDDD:::::A:::::A             A:::::A SSSSSSS     S:::::SSSSSSS     S:::::S"));
  Serial.println(F("B:::::::::::::::::A:::::A               A:::::A D:::::::::::::::DA:::::A               A:::::AS::::::SSSSSS:::::S::::::SSSSSS:::::S"));
  Serial.println(F("B::::::::::::::::A:::::A                 A:::::AD::::::::::::DDDA:::::A                 A:::::S:::::::::::::::SSS:::::::::::::::SS "));
  Serial.println(F("BBBBBBBBBBBBBBBBAAAAAAA                   AAAAAADDDDDDDDDDDDD  AAAAAAA                   AAAAAASSSSSSSSSSSSSSS   SSSSSSSSSSSSSSS   "));
   
}

void cmdResponse(uint8_t _fcncode, uint8_t params[], uint8_t bytesread){
  
  uint32_t tmp_tlmctrl = 0x00001000;
  uint8_t tmp_tlm_len = 0;
  tmp_tlm_len = extractFromTlm(tmp_tlmctrl, incomingByte, tmp_tlm_len);
  tmp_tlm_len = extractFromTlm(fcncode, incomingByte, tmp_tlm_len);
  sendTlmMsg( tlm_addr, telemetry_data, tmp_tlm_len);
  
  uint8_t pkt_pos = 0;

  if(_fcncode == CMD_SetTlmCtrl){
    pkt_pos = extractFromTlm(tlmctrl, incomingByte, pkt_pos);

    Serial.print(" TlmCtrlCmd: ");
    Serial.println(tlmctrl);
  }
  else if(_fcncode == CMD_SetCycTime){
    pkt_pos = extractFromTlm(desiredctrltime, incomingByte, pkt_pos);

    Serial.print(" Setting DesiredCycTime to: ");
    Serial.println(desiredctrltime);
  }
  else if(_fcncode == CMD_SetTargetNED){
    float tmp_float = 0.0;
    pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
    target_ned(0) = tmp_float;
    pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
    target_ned(1) = tmp_float;
    pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
    target_ned(2) = tmp_float;

    Serial.print(" Setting TargetNED to: [");
    Serial.print(target_ned(0));
    Serial.print("; ");
    Serial.print(target_ned(2));
    Serial.print("; ");
    Serial.print(target_ned(2));
    Serial.println("]");
  // target_ned = imu::Vector<3>(1.0, 0.0, 0.0);
  }
  else if(_fcncode == CMD_SetIMU2BODY){
    float tmp_float, tmp_float1, tmp_float2, tmp_float3;
    pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
    pkt_pos = extractFromTlm(tmp_float1, incomingByte, pkt_pos);
    pkt_pos = extractFromTlm(tmp_float2, incomingByte, pkt_pos);
    pkt_pos = extractFromTlm(tmp_float3, incomingByte, pkt_pos);
    quat_imu2body = imu::Quaternion(tmp_float, tmp_float1, tmp_float2, tmp_float3);

    Serial.print(" Setting IMU2Body to: [");
    Serial.print(tmp_float);
    Serial.print("; ");
    Serial.print(tmp_float);
    Serial.print("; ");
    Serial.print(tmp_float);
    Serial.print("; ");
    Serial.print(tmp_float);
    Serial.println("]");
  }
  else if(_fcncode == CMD_SetServoEnable){
    extractFromTlm(ServoEnableFlg, incomingByte, pkt_pos);

    Serial.print(" Setting ServoEnable to: ");
    Serial.println(incomingByte[9]);
  }
  else if(_fcncode == CMD_SetRWEnable){
    uint8_t tmp_uint8 = 0;
    Serial.print(" Setting RWEnable to: ");
    pkt_pos = extractFromTlm(tmp_uint8, incomingByte, pkt_pos);
    Serial.println(tmp_uint8);
  }
  else if(_fcncode == CMD_RequestTlmPt){
    Serial.print(" Sending TlmCtrl: ");
    Serial.println(tmp_tlmctrl);
    
    uint32_t tmp_tlmctrl = 0;
    uint8_t tmp_tlm_len = 0;
    pkt_pos = extractFromTlm(tmp_tlmctrl, incomingByte, pkt_pos);

    tmp_tlm_len = compileTLM(tmp_tlmctrl);
    sendTlmMsg( tlm_addr, telemetry_data, tmp_tlm_len);

  }
  else if(_fcncode == CMD_SendTestPkt){
    Serial.print(" Sending test packet");
    
    uint8_t telemetry_data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    sendTlmMsg( tlm_addr, telemetry_data, 0);
  }
  else if(_fcncode == CMD_SetTlmAddr){
    pkt_pos = extractFromTlm(tlm_addr, incomingByte, pkt_pos);
    Serial.print(" Setting tlm addr:");
    Serial.println(tlm_addr);
  }
  else if(_fcncode == CMD_SetElPolarity){
    pkt_pos = extractFromTlm(El_Cmd_Polarity, incomingByte, pkt_pos);
    Serial.print(" El cmd polarity:");
    Serial.println(El_Cmd_Polarity);
  }
  else if(_fcncode == CMD_SetMode){
    pkt_pos = extractFromTlm(op_mode, incomingByte, pkt_pos);
    Serial.print(" Mode: ");
    Serial.println(op_mode);
    
    if(op_mode == MODE_Manual)
      pkt_pos = extractFromTlm(man_el_ang, incomingByte, pkt_pos);
      Serial.print(" El: ");
      Serial.println(man_el_ang);
  }
 else{
    Serial.print("Unrecongized command...");
 }
  
}

void getInput(){
// checks if the xbee has received a message and processes it
//     if it has
  
  // check if there's data to be read

  if((pkt_type = readMsg(1)) == 0){
    Serial.println("Read something else, reading again");
    pkt_type = readMsg(1);
  }
 
  if (pkt_type > -1 ) {

    // print info about the packet
    printPktInfo();

    // process based on command or telemetry packet
    if(pkt_type){
      bytesread = readCmdMsg(incomingByte, fcncode);
      if(bytesread > 0){
        // response to command
        cmdResponse(fcncode, incomingByte, bytesread);
        cmdrcvdcnt++;
      }
    }
    else{
      // read the telemetry message, don't currently have 
      //  anything to do with it
      bytesread = readTlmMsg(incomingByte);
    }
  } else if(pkt_type < -1){
    Serial.print("Failed to read xbee with code: ");
    Serial.println(pkt_type);
  }
}
// ***********************************
// Setup
void setup() {
  
	////// Initalize Sensors
	
	// begin the serials
	Serial.begin(250000);		// debugging output       
	Serial3.begin(9600);  	// xbee

	// print the splash screen
	BADASS();

	// initalize the BNO
	Serial.print("Starting BNO initialization...");
	if(!bno.begin(bno.OPERATION_MODE_NDOF))
	{
		
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.println(" Failed!");
		
		initstatus |= (INITMASK_BNO & 0xFFFF);
	}
	else{
		Serial.println(" Initalized!");
		
		// set to absolute attitude mode
		bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
		
		delay(100);
		
		/* Display some basic information on this sensor */
		displaySensorDetails();
		
		//bno.setExtCrystalUse(true);
	}

 Serial.print("Starting BNO2 initialization...");
 if(!bno2.begin(bno2.OPERATION_MODE_NDOF))
  {
    
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println(" Failed!");
    
    initstatus |= (INITMASK_BNO & 0xFFFF);
  }
  else{
    Serial.println(" Initalized!");
    
    // set to absolute attitude mode
    bno2.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
    
    delay(100);
    
    /* Display some basic information on this sensor */
    displaySensorDetails2();
    
    //bno.setExtCrystalUse(true);
  }

	// the ads does not return a status when started, so we can't 
	Serial.print("Starting ADS initialization...");
	ads.begin();
    ads.setGain(GAIN_TWOTHIRDS);
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
	Serial.println(" Initalized!"); 


	// initalize the SD card
	Serial.print("Starting SD initialization...");
	pinMode(SD_PIN, OUTPUT);

	if (!SD.begin(SD_PIN)) {
		Serial.println(" Failed!");
		initstatus |= (INITMASK_SD & 0xFFFF);
	}
	else{
		Serial.println(" Initalized!");
		
		// find the next free filename
		int n = 0;
		do{
			snprintf(filename, sizeof(filename), "data%03d.txt", n); // includes a three-digit sequence number in the file name
			n++;
		}
		while(SD.exists(filename));

		// open a log file
		logFile = SD.open(filename, FILE_WRITE);

		// if the file opened okay, write to it:
		if (logFile) {
			Serial.print(filename);
			Serial.println(" opened...");
			logFile.println("BADASS inialize");
		} else {
			// if the file didn't open, print an error:
			Serial.println("error opening ");
			Serial.println(filename);
		}
	}

	// initalize MCP9808_1
    Serial.print("Starting MCP9808_1 initialization...");
	if (tempsensor1.begin(0x1A)) {
		Serial.println(" Initalized!");
	} else {
		Serial.println(" Failed!");
		initstatus |= (INITMASK_MCP9808_1 & 0xFFFF);
	}
	
	// initalize MCP9808_2
	Serial.print("Starting MCP9808_2 initialization...");
	if (tempsensor2.begin(0x1B)) {
		Serial.println(" Initalized!");
	} else {
		Serial.println(" Failed!");
		initstatus |= (INITMASK_MCP9808_2 & 0xFFFF);
	}
	
	// initalize bme
	Serial.print("Starting BME initialization...");
	if (bme.begin(0x76)) {
		Serial.println(" Initalized!");
	} else {
		Serial.println(" Failed!");
		initstatus |= (INITMASK_BME & 0xFFFF);
	}
	
	// initalize xbee
	Serial.println("Starting XBee initialization...");
	if(!InitXBee(xbee_addr, xbee_PanID, Serial3)){
		Serial.println(" Initalized!");
	} else {
		Serial.println(" Failed!");
		initstatus |= (INITMASK_xbee & 0xFFFF);
	}
  /*
	// initalize servo
  Serial.print("Starting servo initialization...");
	if(!servo1.attach(ServoPIN,900,2100)){
		Serial.println(" Initalized!");
		//  move to zero position
		servo1.write(90);
	} else {
		Serial.println(" Failed!");
		initstatus |= (INITMASK_servo & 0xFFFF);
	}
	*/
	// normalize the target
	target_ned.normalize();
	x_axis.normalize();
	y_axis.normalize();
	z_axis.normalize();

	Serial.print("Initialization finished with status: ");
	Serial.println(initstatus);
 
  Serial.print("Starting in mode: ");
  Serial.println(op_mode);
  
  // send init packet
  uint8_t tlm_pos = addIntToTlm(TLMMask_InitStat & 0xFFFFFFFF, telemetry_data, (uint8_t) 0);
  tlm_pos = addIntToTlm(initstatus, telemetry_data, tlm_pos);
  sendTlmMsg( tlm_addr, telemetry_data, tlm_pos);
 
}

// ***********************************
// Loop
void loop() {

	// always respond to commands as soon as we receive them
	// get any input packets
	getInput();
  
	// calculate 
  comm_cycle_time = millis() - comm_cycle_start;
  IMUlog_cycle_time = millis() - IMUlog_cycle_start;
	HKlog_cycle_time = millis() - HKlog_cycle_start;
  ctrl_cycle_time = millis() - ctrl_cycle_start;

	// comm cycle
	if(comm_cycle_time > desiredcommtime){
		comm_cycle_start = millis();
    
		// compile telemetry
		tlm_len = compileTLM(tlmctrl);

		// send the packet
		sendTlmMsg( tlm_addr, telemetry_data, tlm_len);
	}
	
	// log IMU cycle
	if(IMUlog_cycle_time > desiredIMUlogtime){
		IMUlog_cycle_start = millis();
   
		// log IMU related data
		logData((uint32_t) 0x000001C4);
	}
	
	// log hk cycle
	if(HKlog_cycle_time > desiredHKlogtime){
		HKlog_cycle_start = millis();
   
		// log all data
		logData(TLMMask_All);
	}
	
	// only start next cycle if we've waited long enough
	if(ctrl_cycle_time > desiredctrltime){
		ctrl_cycle_start = millis();
    
		temp1 = tempsensor1.readTempC();
		temp2 = tempsensor2.readTempC();
		tempbme = bme.readTemperature();
		pres = bme.readPressure() / 100.0F;
		alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
		humid = bme.readHumidity();

		raw_current = ads.readADC_SingleEnded(ADS_CURRENT_PIN);
		raw_voltage = ads.readADC_SingleEnded(ADS_VOLTAGE_PIN);

		// get calibration status
		bno.getCalibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
    bno2.getCalibration(&sys_cal2, &gyro_cal2, &accel_cal2, &mag_cal2);
    
    if(sys_cal2 > sys_cal){
      // get BNO data 
      Serial.print("Using BNO2 with cal: ");
      Serial.println(sys_cal2);
    }
    else{
      Serial.print("Using BNO with cal ");
      Serial.println(sys_cal);
    }

    if(sys_cal2 > sys_cal){
		  // get BNO data 
		  bno2.getEvent(&event);
    }
    else{
      bno.getEvent(&event);
    }
    
		// get euler angles
    if(sys_cal2 > sys_cal){
      euler = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);
    }
    else{
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    }
    
		// get quaternion
    if(sys_cal2 > sys_cal){
      quat_ned2imu = bno2.getQuat();
      quat_ned2imu2 = bno2.getQuat();
    }
    else{
      quat_ned2imu = bno.getQuat();
    }

		// form ned2body quaternion
		quat_ned2body = quat_ned2imu * quat_imu2body;

		// calculate target in body frame
		target_body = quat_ned2body.rotateVector(target_ned);
		target_body.normalize();

		// calculate pointing error
		calcPtErr(target_body, x_axis, y_axis, pterr_az, pterr_el);

		// if we're in manual mode, override the calculated angle
		if(op_mode == MODE_Manual){
			//for now
			cmd_az_ang = pterr_az;
			cmd_el_ang = man_el_ang;
		} else {
			cmd_az_ang = pterr_az;
			cmd_el_ang = pterr_el;
		}
		
		// enforce actuator stops before commanding
		enforcestops(cmd_az_ang, cmd_el_ang);
		
		// map command onto servo range
		if(El_Cmd_Polarity){
			servo_cmd = map(cmd_el_ang*180/PI,-90,90,min_el_ang,max_el_ang);
		} else {
			servo_cmd = map(cmd_el_ang*180/PI,-90,90,max_el_ang,min_el_ang);
		}

		// command servo
		if(ServoEnableFlg){
      // if the cal is bad, command back to home
      if(sys_cal2 == 0 && sys_cal == 0){
        Serial.println("Cal bad, commanding to home");
        servo1.write(90);
      }
      else{
        servo1.write(servo_cmd);
      }
		}


	}
}
