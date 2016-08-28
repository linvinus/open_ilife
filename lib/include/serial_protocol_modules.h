
#ifndef _SERIAL_PROTOCOL_MODULES_H_
#define _SERIAL_PROTOCOL_MODULES_H_


#include "serial_protocol.h"

typedef struct {
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
}RobotCFG_t,*RobotCFG_ptr;

RobotCFG_t RobotCFG;
const RobotCFG_ptr const pRobotCFG=&RobotCFG;

typedef struct SerialProtocolCmd_t SerialProtocolCmd_t;

SerialProtocolCmd_t SD_CMDS[]={
  {0,NULL,NULL,NULL,0,NULL,NULL,NULL},                              /*SP_SYSTEM_MESSAGE*/
  {sizeof(RobotCFG_t),&RobotCFG,NULL,NULL,sizeof(RobotCFG_t),&RobotCFG,NULL,NULL},        /*SP_CONFIGURATION*/
  };
  //

uint16_t SD_CMDS_COUNT = sizeof(SD_CMDS)/sizeof(SerialProtocolCmd_t);

#endif /* _SERIAL_PROTOCOL_MODULES_H_ */
