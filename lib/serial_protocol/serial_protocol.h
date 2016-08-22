#ifndef _SERIAL_PROTOCOL_H_
#define _SERIAL_PROTOCOL_H_

#include <stdint.h>

#include "serial_protocol_config.h"

typedef void (*SD_CALLBACK)(void *arg);

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

#if defined(_SERIAL_PROTOCOL_CHIBIOS_RT_)
void serial_protocol_thread_init(void);
#endif

void serial_protocol_main_loop_iterate(void);
void serial_protocol_get_cmd(uint8_t cmd);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _SERIAL_PROTOCOL_H_ */
