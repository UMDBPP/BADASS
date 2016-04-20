
#include <XBee.h>
#include <Wire.h>
#include "CCSDS.h"
#include "CCSDS_xbee.h"

#define PKT_MAX_LEN 200

#define NUM_TRANS_APIDS 5

// List of AP_ID for forwarding data to ground
uint16_t Transmitted_AP_IDs[NUM_TRANS_APIDS] = {1, 2, 0, 0, 0};

const uint8_t SyncByte[2] = {0x18, 0x01};
const uint8_t RespondSyncByte[2] = {0x18, 0x02};

uint8_t Buff_9002XbeeBuf[PKT_MAX_LEN];
int BytesRead900 = 0;
uint8_t Buff_Xbee2900[PKT_MAX_LEN];
int XbeeBytesRead = 0;
uint32_t _SendCtr = 0;

CCSDS_TlmSecHdr_t TlmHeader;
CCSDS_CmdSecHdr_t CmdHeader;
CCSDS_PriHdr_t PriHeader;

// set XBee address of this transmitter
uint16_t XBee_MY_Addr = 0x0002;

// PAN ID of xbee (must match all xbees)
uint16_t XBee_PAN_ID = 0x0B0B;

uint8_t err_cnt = 0;

int APID = 0;
int PktLen = 0;
int bytesRead = 0;
int BytesinBuffer = 0;

uint32_t tlmctrl = 0x04;//67108864;

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
    Serial.print(CCSDS_RD_APID(PriHeader));
    Serial.print(", SecHdr: ");
    Serial.print(CCSDS_RD_SHDR(PriHeader));
    Serial.print(", Type: ");
    Serial.print(CCSDS_RD_TYPE(PriHeader));
    Serial.print(", Ver: ");
    Serial.print(CCSDS_RD_VERS(PriHeader));
    Serial.print(", SeqCnt: ");
    Serial.print(CCSDS_RD_SEQ(PriHeader));
    Serial.print(", SegFlag: ");
    Serial.print(CCSDS_RD_SEQFLG(PriHeader));
    Serial.print(", Len: ");
    Serial.println(CCSDS_RD_LEN(PriHeader));

    if(CCSDS_RD_TYPE(PriHeader)){
      Serial.print("FcnCode: ");
      Serial.print(CCSDS_RD_FC(CmdHeader));
      Serial.print(", CkSum: ");
      Serial.println(CCSDS_RD_CHECKSUM(CmdHeader));
    }
    else{
      Serial.print("Sec: ");
      Serial.print(CCSDS_RD_SEC_HDR_SEC(TlmHeader));
      Serial.print(", Subsec: ");
      Serial.println(CCSDS_RD_SEC_HDR_SUBSEC(TlmHeader));
    }
    
    
}

void loop() {
  /////// read from xbee send to 900s
  
  // read message from xbee,
  // bytesRead is positive if there was data read
  XbeeBytesRead = readMsg(Buff_Xbee2900,1);
  
  //Serial.print("Xbee read status: ");
  //Serial.println(XbeeBytesRead);
  // If data proccess and send 
  if (XbeeBytesRead > 0) {
  Serial.println();

      PriHeader = *(CCSDS_PriHdr_t*) (Buff_Xbee2900);
      CmdHeader = *(CCSDS_CmdSecHdr_t*) (Buff_Xbee2900+sizeof(CCSDS_PriHdr_t));
      TlmHeader = *(CCSDS_TlmSecHdr_t*) (Buff_Xbee2900+sizeof(CCSDS_PriHdr_t));

      // Convert inital bytes of packet into header structure
      //memcpy(&PriHeader, Buff_Xbee2900, sizeof(CCSDS_PriHdr_t));
      //memcpy(&CmdHeader, Buff_Xbee2900+6, sizeof(CCSDS_CmdSecHdr_t));
      //memcpy(&TlmHeader, Buff_Xbee2900+6, sizeof(CCSDS_TlmSecHdr_t));
    
      Serial.println("Xbee -> Serial: ");
      printPktInfo(PriHeader, CmdHeader, TlmHeader);
        
        // Extract address 
        int AP_ID = CCSDS_RD_APID(PriHeader);
        // Check against desired addresses and send if matches
          if (AP_ID == Transmitted_AP_IDs[0] || AP_ID == Transmitted_AP_IDs[1]){
            Serial.print("Sending: ");
            for (int i = 0; i < XbeeBytesRead; i++){
             // Print to 900s
             Serial.print(Buff_Xbee2900[i],HEX);

             // NOTE: This must be write instead of print, otherwise it'll convert
             //   it to text characters
             Serial2.write(Buff_Xbee2900[i]);
             Serial.print(", ");
             }
          Serial.println();
          }
       
  } 
  else if(XbeeBytesRead > -4) {
    Serial.print("Failed to read pkt with error code ");
    Serial.println(XbeeBytesRead);
    if(err_cnt > 3){
      Serial3.flush();
      delay(3);
      err_cnt = 0;
      Serial.println("Flushing buffer");
    }
    err_cnt++;
  }

  /////// 900s to xbee

  // Read from 900s append to buffer     
  BytesRead900 = Serial2.readBytes(Buff_9002XbeeBuf+BytesinBuffer, Serial2.available());
  
  //Serial.print("Read bytes: ");
  //Serial.print(BytesRead900);
  
  // updating counter
  BytesinBuffer += BytesRead900;

  if(BytesRead900 > 0){
          Serial.print("REad :");
      Serial.println(BytesRead900);
  }
  
  //Serial.print(", BytesinBuf: ");
  //Serial.println(BytesinBuffer);
  
  // Looking for sync bytes 
  for(int i=0; i<BytesinBuffer; i++){

    //PriHeader = *(CCSDS_PriHdr_t*) (Buff_9002XbeeBuf);
    
    if(Buff_9002XbeeBuf[i] == SyncByte[0] & Buff_9002XbeeBuf[i+1] == SyncByte[1]) {

      PriHeader = *(CCSDS_PriHdr_t*) (Buff_9002XbeeBuf+i);
      CmdHeader = *(CCSDS_CmdSecHdr_t*) (Buff_9002XbeeBuf+i+sizeof(CCSDS_PriHdr_t));
      TlmHeader = *(CCSDS_TlmSecHdr_t*) (Buff_9002XbeeBuf+i+sizeof(CCSDS_PriHdr_t));
      
      // Convert inital bytes of packet into header structure
      //memcpy(&PriHeader, Buff_9002XbeeBuf, sizeof(CCSDS_PriHdr_t));
      //memcpy(&CmdHeader, Buff_9002XbeeBuf+6, sizeof(CCSDS_CmdSecHdr_t));
      //memcpy(&TlmHeader, Buff_9002XbeeBuf+6, sizeof(CCSDS_TlmSecHdr_t));

      // display debugging info
      Serial.println("Serial -> Xbee: ");
      printPktInfo(PriHeader, CmdHeader, TlmHeader);
          
      // Get the total length of packet
      int PacketLength = CCSDS_RD_LEN(PriHeader);
      // Extract address 
      int AP_ID = CCSDS_RD_APID(PriHeader);
      
      // If you have a whole packet send it
      if(BytesinBuffer >= PacketLength+i){

        Serial.print("Sending to: ");
        Serial.print(AP_ID);
        Serial.print(" ");
        Serial.print(PacketLength);
        Serial.println(" bytes");
       // for(int ii=0;ii<PacketLength;ii++){
       //   Serial.print(Buff_9002XbeeBuf[i+ii],HEX);
       //   Serial.print(" ,");
       // }
       // Serial.println();

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
    if(Buff_9002XbeeBuf[i] == RespondSyncByte[0] & Buff_9002XbeeBuf[i+1] == RespondSyncByte[1]) {



      PriHeader = *(CCSDS_PriHdr_t*) (Buff_9002XbeeBuf+i);
      CmdHeader = *(CCSDS_CmdSecHdr_t*) (Buff_9002XbeeBuf+i+sizeof(CCSDS_PriHdr_t));
      TlmHeader = *(CCSDS_TlmSecHdr_t*) (Buff_9002XbeeBuf+i+sizeof(CCSDS_PriHdr_t));
      
      // Convert inital bytes of packet into header structure
      //memcpy(&PriHeader, Buff_9002XbeeBuf, sizeof(CCSDS_PriHdr_t));
      //memcpy(&CmdHeader, Buff_9002XbeeBuf+6, sizeof(CCSDS_CmdSecHdr_t));
      //memcpy(&TlmHeader, Buff_9002XbeeBuf+6, sizeof(CCSDS_TlmSecHdr_t));

      // display debugging info
      Serial.println("Serial -> Respond: ");
      printPktInfo(PriHeader, CmdHeader, TlmHeader);
          
      // Get the total length of packet
      int PacketLength = CCSDS_RD_LEN(PriHeader);
      // Extract address 

      Serial.print("Recvd pktlen: ");
      Serial.println(PacketLength);
      
      // If you have a whole packet send it
      if(BytesinBuffer >= PacketLength+i){


        uint8_t _packet_data[PKT_MAX_LEN];
        uint8_t _payload_size = 0;
        
        // declare the header structures
        CCSDS_PriHdr_t _PriHeader;
        CCSDS_TlmSecHdr_t _TlmSecHeader;

        _payload_size = 8+sizeof(_PriHeader)+sizeof(_TlmSecHeader);
      
        // fill primary header fields
        CCSDS_WR_APID(_PriHeader,0x03);
        CCSDS_WR_SHDR(_PriHeader,1);
        CCSDS_WR_TYPE(_PriHeader,0);
        CCSDS_WR_VERS(_PriHeader,0);
        CCSDS_WR_SEQ(_PriHeader,_SendCtr);
        CCSDS_WR_SEQFLG(_PriHeader,0x03);
        CCSDS_WR_LEN(_PriHeader,8+sizeof(_PriHeader)+sizeof(_TlmSecHeader));

                // fill secondary header fields
        CCSDS_WR_SEC_HDR_SEC(_TlmSecHeader,millis()/1000L);
        CCSDS_WR_SEC_HDR_SUBSEC(_TlmSecHeader,millis() % 1000L);

        // copy the primary header
          memcpy(_packet_data, &_PriHeader, sizeof(_PriHeader));
        
          // copy the secondary header
          memcpy(_packet_data+sizeof(_PriHeader), &_TlmSecHeader, sizeof(_TlmSecHeader));
        
          // copy the packet data
          memcpy(_packet_data+sizeof(_PriHeader)+sizeof(_TlmSecHeader), &tlmctrl, 4);
          memcpy(_packet_data+sizeof(_PriHeader)+sizeof(_TlmSecHeader)+4, &_SendCtr, 4);

        Serial.print("Sending  ");
        Serial.print(_payload_size);
        Serial.println(" bytes: ");
        _SendCtr++;
        for(int ii=0; ii < _payload_size; ii++){
          Serial.print( _packet_data[ii]);
          Serial2.write( _packet_data[ii]);
          Serial.print( ", ");
        }
        Serial.println();
        
        // updating counter
        BytesinBuffer -= PacketLength;
        Serial.print(", BytesinBuf: ");
        Serial.println(BytesinBuffer);
        break;

      }
    }
    Serial.print(", BytesinBuf: ");
        Serial.println(BytesinBuffer);
  }
  
  delay(10);

}
