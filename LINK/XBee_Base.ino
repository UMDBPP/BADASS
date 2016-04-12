
#include <XBee.h>
#include <Wire.h>
#include "CCSDS.h"
#include "CCSDS_xbee.h"

#define PKT_MAX_LEN 200

#define NUM_TRANS_APIDS 5

// List of AP_ID for forwarding data to ground
uint16_t Transmitted_AP_IDs[NUM_TRANS_APIDS] = {1, 2, 0, 0, 0};

const uint8_t SyncByte[2] = {0x18, 0x01};

uint8_t Buff_9002XbeeBuf[PKT_MAX_LEN];
int BytesRead900 = 0;
uint8_t Buff_Xbee2900[PKT_MAX_LEN];
int XbeeBytesRead = 0;

CCSDS_TlmSecHdr_t TlmHeader;
CCSDS_CmdSecHdr_t CmdHeader;
CCSDS_PriHdr_t PriHeader;

// set XBee address of this transmitter
uint16_t XBee_MY_Addr = 0x0002;

// PAN ID of xbee (must match all xbees)
uint16_t XBee_PAN_ID = 0x0B0B;

int APID = 0;
int PktLen = 0;
int bytesRead = 0;
int BytesinBuffer = 0;


void setup() {
  // debug
  Serial.begin(250000);          
  // xbee
  Serial3.begin(9600); 
  // radio
  Serial2.begin(9600); 
  
  Serial.println("Base station initalize!");
  
  // initalize xbee
  int XbeeStatus = InitXBee(XBee_MY_Addr, XBee_PAN_ID, Serial3);
  if (!XbeeStatus) {
    Serial.println(F("Xbee initialized!"));

  }
  else {

    Serial.print(F("Xbee failed to initialize with Error code: "));
    Serial.println(XbeeStatus);
  }
  
}


void printPktInfo(CCSDS_PriHdr_t PriHeader, CCSDS_CmdSecHdr_t CmdHeader, CCSDS_TlmSecHdr_t TlmHeader){
    
    Serial.print("APID: ");
    Serial.println(CCSDS_RD_APID(PriHeader));
    Serial.print("SecHdr: ");
    Serial.println(CCSDS_RD_SHDR(PriHeader));
    Serial.print("Type: ");
    Serial.println(CCSDS_RD_TYPE(PriHeader));
    Serial.print("Ver: ");
    Serial.println(CCSDS_RD_VERS(PriHeader));
    Serial.print("SeqCnt: ");
    Serial.println(CCSDS_RD_SEQ(PriHeader));
    Serial.print("SegFlag: ");
    Serial.println(CCSDS_RD_SEQFLG(PriHeader));
    Serial.print("Len: ");
    Serial.println(CCSDS_RD_LEN(PriHeader));

    if(CCSDS_RD_TYPE(PriHeader)){
      Serial.print("Cmd: ");
      Serial.println(CCSDS_RD_FC(CmdHeader));
      Serial.print("CkSum: ");
      Serial.println(CCSDS_RD_CHECKSUM(CmdHeader));
    }
    else{
      Serial.print("Sec: ");
      Serial.println(CCSDS_RD_SEC_HDR_SEC(TlmHeader));
      Serial.print("Subsec: ");
      Serial.println(CCSDS_RD_SEC_HDR_SUBSEC(TlmHeader));
    }
    
    
}

void loop() {
  /////// read from xbee send to 900s
  
  // read message from xbee,
  // bytesRead is positive if there was data read
  XbeeBytesRead = readMsg(Buff_Xbee2900,1);
  
  Serial.print("Xbee read status: ");
  Serial.println(XbeeBytesRead);
  
  // If data proccess and send 
  if (XbeeBytesRead > 0) {
      // Convert inital bytes of packet into header structure
      memcpy(&PriHeader, Buff_Xbee2900, sizeof(CCSDS_PriHdr_t));
      memcpy(&CmdHeader, Buff_Xbee2900+6, sizeof(CCSDS_CmdSecHdr_t));
      memcpy(&TlmHeader, Buff_Xbee2900+6, sizeof(CCSDS_TlmSecHdr_t));
    
      Serial.println("Received from xbee, forwards on serial: ");
      printPktInfo(PriHeader, CmdHeader, TlmHeader);
        
        // Extract address 
        int AP_ID = CCSDS_RD_APID(PriHeader);
        // Check against desired addresses and send if matches
          if (AP_ID == Transmitted_AP_IDs[0] || AP_ID == Transmitted_AP_IDs[1]){
            Serial.print("Sending on serial: ");
            for (int i = 0; i < XbeeBytesRead; i++){
             // Print to 900s
             Serial.print(Buff_Xbee2900[i],HEX);
             Serial2.print(Buff_Xbee2900[i]);
             Serial.print(", ");
             }
          Serial2.println();
          Serial.println();
          }
       
  } 
  //else{
  //  Serial.print("Failed to read pkt with error code ");
  //  Serial.println(XbeeBytesRead);
  //}

  /////// 900s to xbee

  // Read from 900s append to buffer     
  BytesRead900 = Serial2.readBytes(Buff_9002XbeeBuf+BytesinBuffer, Serial2.available());
  
  Serial.print("Read bytes: ");
  Serial.print(BytesRead900);
  
  // updating counter
  BytesinBuffer += BytesRead900;
  
  Serial.print(", BytesinBuf: ");
  Serial.println(BytesinBuffer);
  
  // Looking for sync bytes 
  for(int i=0; i<BytesinBuffer; i++){
    if(Buff_9002XbeeBuf[i] == SyncByte[0] & Buff_9002XbeeBuf[i+1] == SyncByte[1]) {

      Serial.print("synch bytes: ");
      Serial.print(Buff_9002XbeeBuf[i],HEX);
      Serial.print(",  ");
      Serial.println(Buff_9002XbeeBuf[i+i],HEX);
      
      // Convert inital bytes of packet into header structure
      memcpy(&PriHeader, Buff_9002XbeeBuf, sizeof(CCSDS_PriHdr_t));
      memcpy(&CmdHeader, Buff_9002XbeeBuf+6, sizeof(CCSDS_CmdSecHdr_t));
      memcpy(&TlmHeader, Buff_9002XbeeBuf+6, sizeof(CCSDS_TlmSecHdr_t));

      // display debugging info
      Serial.println("Received from serial, forwards on xbee: ");
      printPktInfo(PriHeader, CmdHeader, TlmHeader);
          
      // Get the total length of packet
      int PacketLength = CCSDS_RD_LEN(PriHeader);
      // Extract address 
      int AP_ID = CCSDS_RD_APID(PriHeader);
      
      // If you have a whole packet send it
      if(BytesinBuffer >= PacketLength+i){

        Serial.print("Sending pkt bytes: ");
        Serial.println(PacketLength);

        // send the packet over the xbee
        sendData(AP_ID, Buff_9002XbeeBuf+i, PacketLength);
        
        // Shifting unsent data in the buffer
        memcpy(Buff_9002XbeeBuf, Buff_9002XbeeBuf+i+PacketLength, BytesinBuffer-PacketLength-i);
        
        // updating counter
        BytesinBuffer = BytesinBuffer-PacketLength-i;
        Serial.print(", BytesinBuf: ");
        Serial.println(BytesinBuffer);
      }
     break;
    }     
  }
  delay(1000);

}
