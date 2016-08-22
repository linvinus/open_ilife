#ifndef _SERIAL_PROTOCOL_H_
#define _SERIAL_PROTOCOL_H_

#include "ch.h"
#include "hal.h"

typedef void (*SD_CALLBACK)(void *arg);

//~ typedef enum SerialPacketType_t SerialPacketType_t;

typedef struct SerialProtocolCmd_t SerialProtocolCmd_t;

struct SerialProtocolCmd_t{
  uint16_t rx_data_size;
  void *rx_data;
  SD_CALLBACK rx_callback;
  void *rx_arg;
  uint16_t tx_data_size;
  void *tx_data;
  SD_CALLBACK tx_callback;
  void *tx_arg;
};


extern SerialProtocolCmd_t SD_CMDS[];
extern uint16_t SD_CMDS_COUNT;

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

void serial_protocol_init(SerialDriver *sdp);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _SERIAL_PROTOCOL_H_ */
