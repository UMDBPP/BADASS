#ifndef _badass_interface_h_
#define _badass_interface_h_


////// Commanding 
// these values define the fcncode corresponding to each command
// 0x01 doesnt work
#define CMD_SetTlmCtrl     	0x01
#define CMD_SetCycTime     	0x02
#define CMD_SetTargetNED 	0x03
#define CMD_SetIMU2BODY 	0x04
#define CMD_SetServoEnable  0x05
#define CMD_SetRWEnable 	0x06 // Not yet implemented
#define CMD_RequestTlmPt	0x07
#define CMD_SendTestPkt 	0x08
#define CMD_SetTlmAddr 		0x09
#define CMD_SetElPolarity 	0x0A
#define CMD_SetMode 		0x0B

////// Telemetry 
// these values define positions in a bitfield which control wether or 
//  not the telemetry point will be output in the nominal telemetry packet
#define TLMMask_TlmCtrl       0x00000001  // 2^0
#define TLMMask_TargetNED     0x00000002  // 2^1
#define TLMMask_BNOCal        0x00000004  // 2^2
#define TLMMask_EulerAng      0x00000008  // 2^3
#define TLMMask_q_imu2body    0x00000010  // 2^4
#define TLMMask_q_ned2imu     0x00000020  // 2^5
#define TLMMask_q_ned2body    0x00000040  // 2^6
#define TLMMask_v_targetbody  0x00000080  // 2^7
#define TLMMask_AzElErr       0x00000100  // 2^8
#define TLMMask_ElCmd         0x00000200  // 2^9
#define TLMMask_CycleTime     0x00000400  // 2^10
#define TLMMask_CmdRcvd       0x00000800  // 2^11
#define TLMMask_DesiredCycTime  0x00001000  // 2^12
#define TLMMask_Temp1           0x00002000  // 2^14
#define TLMMask_Temp2         0x00004000  // 2^15
#define TLMMask_TempBME       0x00008000  // 2^16
#define TLMMask_Pres          0x00010000  // 2^17
#define TLMMask_PresAlt       0x00020000  // 2^18
#define TLMMask_Humid         0x00040000  // 2^19
#define TlmMask_Current       0x00080000  // 2^20
#define TLMMask_Volt          0x00100000  // 2^21
#define TLMMask_InitStat      0x00200000  // 2^22
#define TLMMask_MsgSent         0x00400000  // 2^23
#define TLMMask_MsgRcvd         0x00800000  // 2^24
#define TLMMask_Mode          0x01000000  // 2^25
#define TLMMask_LINKSendCtr   0x02000000  // 2^26

#define TLMMask_All   0xFFFFFFFF  // 2^26


#endif  /* _badass_interface_h_ */
