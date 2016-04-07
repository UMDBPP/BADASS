// ***********************************
// Includes

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BNO055.h"
#include <Servo.h> 
#include <SPI.h>
#include <SD.h>

#include "CCSDS.h"
#include "CCSDS_xbee.h"


#define SD_PIN 53

// ***********************************
// Initalizations

// initalize the log file
File logFile;

// initalize the servo motor
Servo servo1;

// initalize the BNO
Adafruit_BNO055 bno = Adafruit_BNO055(0,0x29);

////// Telemetry 
// these values define positions in a bitfield which control wether or 
//  not the telemetry point will be output in the nominal telemetry packet
uint32_t tlmctrl_MASK =      0x00000001;
uint32_t target_ned_MASK =   0x00000002;
uint32_t bno_cal_MASK =      0x00000004;
uint32_t euler_ang_MASK =    0x00000008;
uint32_t q_imu2body_MASK =   0x00000010;
uint32_t q_ned2imu_MASK =    0x00000020;
uint32_t q_ned2body_MASK =   0x00000040;
uint32_t v_targetbody_MASK = 0x00000080;
uint32_t azel_err_MASK =     0x00000100;
uint32_t el_cmd_MASK =       0x00000200;
uint32_t cycle_time_MASK =   0x00000400;
uint32_t cmd_rcvd_MASK =     0x00000800;
uint32_t desired_cyc_time_MASK = 0x00001000;
uint32_t cmdecho_MASK =      0x00001000;

// this bitfield, and the masks above, define which values get output in
//  telemetry 
uint32_t tlmctrl = 0b0000111110000111;

////// Commanding 
// these values define the fcncode corresponding to each command
uint8_t set_tlmctrl_CMD = 0x01;
uint8_t set_cyctime_CMD = 0x02;
uint8_t set_target_ned_CMD = 0x03;
uint8_t set_imu2body_CMD = 0x04;
uint8_t set_servoenable_CMD = 0x05;
uint8_t set_rwenable_CMD = 0x06;
uint8_t requesttlm_CMD = 0x07;
uint8_t sendtestpkt_CMD = 0x08;

// Initial parameters
//  These parameters control program execution and may be changed via 
//    commands during execution

// execution rate of the program
int desiredcyclestime = 2000; // [ms]

// command the servo
bool cmd_servo_flag = false;

// target in the NED frame
imu::Vector<3> target_ned = imu::Vector<3>(1.0, 0.0, 0.0);

// transform between imu and body frame
imu::Quaternion quat_imu2body = imu::Quaternion(0.7071, 0.0, 0.0, 0.7071);

// coordinate axes in the body frame
imu::Vector<3> x_axis = imu::Vector<3>( 1.0, 0.0, 0.0);
imu::Vector<3> y_axis = imu::Vector<3>( 0.0, 1.0, 0.0);
imu::Vector<3> z_axis = imu::Vector<3>( 0.0, 0.0, 1.0);

// delete these
int prevmillis_tmp = 0;
int prev_cycle_start_time = 0;

// Hardcoded parameters
#define CMD_HDR_LEN 8
#define TLM_HDR_LEN 12

// Program Memory
// These values are calculated/generated during the exectuion of
//  the program
int cycle_start_time = 0;
int servo_cmd = 0;
uint8_t incomingByte[100];
uint8_t bytes_available = 0;
uint8_t tlm_pos = 0;
uint8_t telemetry_data[200];
uint16_t tlm_seq_cnt = 0;

CCSDS_TlmPkt_t TlmHeader;
CCSDS_CmdPkt_t CmdHeader;

uint8_t tmp_uint8 = 0;
uint8_t tmp_uint16 = 0;
uint32_t tmp_uint32 = 0;
float tmp_float = 0.0;
float tmp_float1 = 0.0;
float tmp_float2 = 0.0;
float tmp_float3 = 0.0;

uint16_t xbee_addr = 01;
uint16_t xbee_PanID = 0x0B0B;

uint8_t sys_cal = 0, gyro_cal = 0, accel_cal = 0, mag_cal = 0;
uint8_t sys_stat = 0, st_res = 0, sys_err = 0;
int i = 0;
float az = 0, el = 0;
int cycle_time;

int fcncode = 0;
int APID = 0;
int PktType = 0;

sensors_event_t event; 

// target is defined in NED frame (North, East, Down)
imu::Vector<3> target_body;
imu::Vector<3> in_plane_tgt;
imu::Vector<3> euler;

imu::Quaternion quat_ned2body;
imu::Quaternion quat_ned2imu;

// ***********************************
// Functions

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void cart2spher(imu::Vector<3> vec, float *theta, float *phi){
  *theta = atan2(vec(1),vec(0));
  *phi = atan2(sqrt(pow(vec(0),2)+pow(vec(1),2)),vec(2));
}

void enforcestops(float *az, float *el){
  // enforce El between +/- 90
  if(*el < -(float)PI/2){
    *el = -(float)PI/2;
  }
  else if(*el*2 > (float)PI){
    *el = (float)PI/2;
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
    
void compileTLM(){
  tlm_pos = 0;
  
  tlm_seq_cnt++;
  
  // create header
  CCSDS_WR_APID(TlmHeader.PriHdr,2);
  CCSDS_WR_SHDR(TlmHeader.PriHdr,1);
  CCSDS_WR_TYPE(TlmHeader.PriHdr,0);
  CCSDS_WR_VERS(TlmHeader.PriHdr,0);
  CCSDS_WR_SEQ(TlmHeader.PriHdr,tlm_seq_cnt);
  CCSDS_WR_SEQFLG(TlmHeader.PriHdr,00);
  CCSDS_WR_SEC_HDR_SEC(TlmHeader.SecHdr,millis()/1000L);
  CCSDS_WR_SEC_HDR_SUBSEC(TlmHeader.SecHdr,millis() % 1000L);
  
  tlm_pos = 12;

  // telemetry compilation
   if(tlmctrl & tlmctrl_MASK){
      tlm_pos = addIntToTlm(tlmctrl, telemetry_data, tlm_pos);
      
      Serial.print(" Set tlmctrl: ");
      Serial.print(tlmctrl);
      

    }
    
   logFile.print(tlmctrl);
   logFile.print(", ");
      
   if(tlmctrl & target_ned_MASK){
    tlm_pos = addFloatToTlm( target_ned(0), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_ned(1), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_ned(2), telemetry_data, tlm_pos);
    
    Serial.print(" Set target_ned: ");
    Serial.print(target_ned(0));
    Serial.print(" ");
    Serial.print(target_ned(1));
    Serial.print(" ");
    Serial.print(target_ned(2));
  }
  
  logFile.print(target_ned(0));
  logFile.print(", ");
  logFile.print(target_ned(1));
  logFile.print(", ");
  logFile.print(target_ned(2));
  logFile.print(", ");
    
  if(tlmctrl & bno_cal_MASK){
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

  logFile.print(sys_cal);
  logFile.print(", ");
  logFile.print(gyro_cal);
  logFile.print(", ");
  logFile.print(accel_cal);
  logFile.print(", ");
  logFile.print(mag_cal);
  logFile.print(", ");
    
  if(tlmctrl & euler_ang_MASK){
    tlm_pos = addFloatToTlm( euler.x(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( euler.y(), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( euler.z(), telemetry_data, tlm_pos);

    Serial.print(" euler ang: ");
    Serial.print("[");
    Serial.print(euler.x());
    Serial.print("; ");
    Serial.print(euler.y());
    Serial.print("; ");
    Serial.print(euler.z());
    Serial.print("]");

  }
  
  logFile.print(euler.x());
  logFile.print(", ");
  logFile.print(euler.y());
  logFile.print(", ");
  logFile.print(euler.z());
  logFile.print(", ");
    
  if(tlmctrl & q_imu2body_MASK){
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
  
  logFile.print(quat_imu2body.x());
  logFile.print(", ");
  logFile.print(quat_imu2body.y());
  logFile.print(", ");
  logFile.print(quat_imu2body.z());
  logFile.print(", ");
  logFile.print(quat_imu2body.w());
  logFile.print(", ");
    
  if(tlmctrl & q_ned2imu_MASK){
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

  logFile.print(quat_ned2imu.x());
  logFile.print(", ");
  logFile.print(quat_ned2imu.y());
  logFile.print(", ");
  logFile.print(quat_ned2imu.z());
  logFile.print(", ");
  logFile.print(quat_ned2imu.w());
  logFile.print(", ");
  
  if(tlmctrl & q_ned2body_MASK){
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
  
  logFile.print(quat_ned2body.x());
  logFile.print(", ");
  logFile.print(quat_ned2body.y());
  logFile.print(", ");
  logFile.print(quat_ned2body.z());
  logFile.print(", ");
  logFile.print(quat_ned2body.w());
  logFile.print(", ");
  
  if(tlmctrl & v_targetbody_MASK){
    tlm_pos = addFloatToTlm( target_body(0), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_body(1), telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( target_body(2), telemetry_data, tlm_pos);
    
    Serial.print(" target_body: ");
    Serial.print("[");
    Serial.print(target_body(0));
    Serial.print("; ");
    Serial.print(target_body(1));
    Serial.print("; ");
    Serial.print(target_body(2));
    Serial.print("]");

  }
  
  logFile.print(target_body(0));
  logFile.print(", ");
  logFile.print(target_body(1));
  logFile.print(", ");
  logFile.print(target_body(2));
  logFile.print(", ");
  
  if(tlmctrl & azel_err_MASK){
    tlm_pos = addFloatToTlm( az, telemetry_data, tlm_pos);
    tlm_pos = addFloatToTlm( el, telemetry_data, tlm_pos);
    
    Serial.print(" az_el err: ");
    Serial.print("[");
    Serial.print(az,4);
    Serial.print("; ");
    Serial.print(el,4);
    Serial.print("]");
  }
  
  logFile.print(az);
  logFile.print(", ");
  logFile.print(el);
  logFile.print(", ");
    
  if(tlmctrl & el_cmd_MASK){
    tlm_pos = addIntToTlm(servo_cmd, telemetry_data, tlm_pos);
    
    Serial.print(" Cmd: ");
    Serial.print(servo_cmd);
  }
  
  logFile.print(servo_cmd);
  logFile.print(", ");
    
  if(tlmctrl & cycle_time_MASK){
    tlm_pos = addIntToTlm(cycle_time, telemetry_data, tlm_pos);

    Serial.print(" CycTime: ");
    Serial.print(cycle_time);
  }
  
  logFile.print(cycle_time);
  logFile.print(", ");
    
  if(tlmctrl & cmd_rcvd_MASK){
    
    Serial.print(" Rcvd: ");
    for(i = 0; i < bytes_available; i++){
      Serial.print(incomingByte[i]);
      Serial.print(" ");
    }
  }

  if(tlmctrl & desired_cyc_time_MASK){
    tlm_pos = addIntToTlm(desiredcyclestime, telemetry_data, tlm_pos);

    Serial.print(" DesiredCycTime: ");
    Serial.print(desiredcyclestime);
  }

  logFile.print(desiredcyclestime);
  logFile.print(", ");
  
  if(tlmctrl & cmdecho_MASK){
    tlm_pos = addIntToTlm(fcncode, telemetry_data, tlm_pos);

    Serial.print(" FcnCodeEcho: ");
    Serial.print(fcncode);
  }

  logFile.print(fcncode);
  logFile.print(", ");
  
  if(tlmctrl){
    Serial.println(" ");
  }

  CCSDS_WR_LEN(TlmHeader.PriHdr,tlm_pos);
  
  memcpy(telemetry_data, &TlmHeader, sizeof(TlmHeader));

  //for(i=0;i<sizeof(CCSDS_TlmPkt_t);i++){
  //  telemetry_data[i] = TlmHeader_u.hdrbytes[i];
  //}
  
}

void cmdResponse(){
  uint8_t pkt_pos = CMD_HDR_LEN;
  
  if(fcncode == set_tlmctrl_CMD){
        pkt_pos = extractFromTlm(tlmctrl, incomingByte, pkt_pos);

        Serial.print(" TlmCtrlCmd: ");
        Serial.println(tlmctrl);
      }
      else if(fcncode == set_cyctime_CMD){
        pkt_pos = extractFromTlm(desiredcyclestime, incomingByte, pkt_pos);
        
        Serial.print(" Setting DesiredCycTime to: ");
        Serial.println(desiredcyclestime);
      }
      else if(fcncode == set_target_ned_CMD){
        pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
        target_ned(0) = tmp_float;
        pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
        target_ned(1) = tmp_float;
        pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
        target_ned(2) = tmp_float;
        
        Serial.print(" Setting TargetNED to: [");
        Serial.println(target_ned(0));
        Serial.print("; ");
        Serial.println(target_ned(2));
        Serial.print("; ");
        Serial.println(target_ned(2));
        Serial.print("]");
        // target_ned = imu::Vector<3>(1.0, 0.0, 0.0);
      }
      else if(fcncode == set_imu2body_CMD){
        
        pkt_pos = extractFromTlm(tmp_float, incomingByte, pkt_pos);
        pkt_pos = extractFromTlm(tmp_float1, incomingByte, pkt_pos);
        pkt_pos = extractFromTlm(tmp_float2, incomingByte, pkt_pos);
        pkt_pos = extractFromTlm(tmp_float3, incomingByte, pkt_pos);
        quat_imu2body = imu::Quaternion(tmp_float, tmp_float1, tmp_float2, tmp_float3);
        
        Serial.print(" Setting IMU2Body to: [");
        Serial.println(tmp_float);
        Serial.print("; ");
        Serial.println(tmp_float);
        Serial.print("; ");
        Serial.println(tmp_float);
        Serial.print("; ");
        Serial.println(tmp_float);
        Serial.print("]");
      }
      else if(fcncode == set_servoenable_CMD){
        extractFromTlm(cmd_servo_flag, incomingByte, CMD_HDR_LEN);
        
        Serial.print(" Setting ServoEnable to: ");
        Serial.println(incomingByte[9]);
      }
      else if(fcncode == set_rwenable_CMD){
        Serial.print(" Setting ServoEnable to: ");
        pkt_pos = extractFromTlm(tmp_uint8, incomingByte, pkt_pos);
        Serial.println(tmp_uint8);
      }
      else if(fcncode == requesttlm_CMD){
        pkt_pos = extractFromTlm(tmp_uint32, incomingByte, pkt_pos);
        
        Serial.print(" Setting RequestTlm to: [");
        Serial.println(tmp_uint32);
      }
      else if(fcncode == sendtestpkt_CMD){
        Serial.print(" Sending test packet");
      }
}

void calcPtErr(imu::Vector<3> target_body, imu::Vector<3> x_axis, imu::Vector<3> y_axis){
	
	// calculate az error
    az = atan2(target_body(1),target_body(0));

    // calculate el error
    in_plane_tgt = normtoplane(y_axis, target_body);
    in_plane_tgt.normalize();
    el = acos(in_plane_tgt.dot(x_axis));
    if(in_plane_tgt(2) < 0){
      el = - el;
    }
}

// ***********************************
// Setup
void setup() {
  //Serial.begin(250000);           // set up Serial library at 250000 bps

  // begin the serials
  // usb
  Serial.begin(250000);          
  //xbee
  Serial3.begin(9600); 

  Serial.println("BADASS Initalize!");
  
  // initalize the servo and move to zero position
  servo1.attach(10,900,2100);
  servo1.write(90); 
   
  // initalize the BNO
  if(!bno.begin(bno.OPERATION_MODE_NDOF))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  else{
    Serial.print("BNO Initalized");
  }
  
  // set to absolute attitude mode
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  
  delay(1000);
    
  /* Display some basic information on this sensor */
  displaySensorDetails();

  //bno.setExtCrystalUse(true);
  
  // initalize the SD card
  Serial.print("Initializing SD card...");
  pinMode(SD_PIN, OUTPUT);

  if (!SD.begin(SD_PIN)) {
    Serial.println("SD initialization failed!");
  }
  else{
    Serial.println("SD initialization done.");
  }

  // find the next free filename
  char filename[16]; // make it long enough to hold your longest file name, plus a null terminator
  int n = 0;
  do{
    snprintf(filename, sizeof(filename), "data%03d.txt", n); // includes a three-digit sequence number in the file name
    n++;
    Serial.println(filename);
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

  // initalize xbee
  InitXBee(xbee_addr, xbee_PanID, Serial3);

  // normalize the target
  target_ned.normalize();
  x_axis.normalize();
  y_axis.normalize();
  z_axis.normalize();
    
}


// ***********************************
// Loop
void loop() {
  cycle_time = millis() - cycle_start_time;

  // only start next cycle if we've waited long enough
  if(cycle_time > desiredcyclestime){
    cycle_start_time = millis();

    // command handling
    bytes_available = Serial.available();
    if (bytes_available > 8) {
      // read the incoming byte:
      Serial.readBytes(incomingByte, Serial.available());

      //for(i=0;i<sizeof(CCSDS_CmdPkt_t);i++){
      //  CmdHeader_u.hdrbytes[i] = incomingByte[i];
      //}
      //PktType = CCSDS_RD_TYPE(CmdHeader.PktHdr.PriHdr);
      //fcncode = CCSDS_RD_FC(CmdHeader.PktHdr.SecHdr);
      //APID = CCSDS_RD_APID(CmdHeader.PktHdr.PriHdr);
      
      // copy header into structure
      memcpy(&CmdHeader, incomingByte, sizeof(CmdHeader));

      // extract header values
      PktType = CCSDS_RD_TYPE(CmdHeader.PriHdr);
      fcncode = CCSDS_RD_FC(CmdHeader.SecHdr);
      APID = CCSDS_RD_APID(CmdHeader.PriHdr);
      
      Serial.print("Received pkt type: ");
      Serial.print(PktType);
      Serial.print(" FcnCode: ");
      Serial.print(fcncode);
        
      cmdResponse();

    }

    // get calibration status
    bno.getCalibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);

    // get BNO data 
    bno.getEvent(&event);

    // get euler angles
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    // get quaternion
    quat_ned2imu = bno.getQuat();

    // form ned2body quaternion
    quat_ned2body = quat_ned2imu* quat_imu2body;

    // calculate target in body frame
    target_body = quat_ned2body.rotateVector(target_ned);
    target_body.normalize();

    // calculate pointing error
    calcPtErr(target_body, x_axis, y_axis);

    // enforce actuator stops before commanding
    enforcestops(&az, &el);

    // map command onto servo range
    servo_cmd = map(el*180/PI,-90,90,180,0);

    // command servo
    if(cmd_servo_flag){
      servo1.write(servo_cmd);
    }

  compileTLM();

  sendData( (uint8_t) 2, telemetry_data, tlm_pos);
  //sendData( (uint8_t) 3, telemetry_data, tlm_pos);
  }
}