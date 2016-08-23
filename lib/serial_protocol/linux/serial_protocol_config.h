#ifndef _SERIAL_PROTOCOL_CONFIG_H_
#define _SERIAL_PROTOCOL_CONFIG_H_


//#define _SERIAL_PROTOCOL_CHIBIOS_RT_

//#include "ch.h"
//#include "hal.h"
/* sd_get_timeout(ms)
 * return >= 0 - value
 * return <  0 - error
 * */
#define sd_get_timeout(time_ms) sd_read_byte(time_ms)

/*
 * return 0 - SUCCSESS
 * */
#define sd_put_timeout(byte,time_ms) sd_write_byte(byte,time_ms)
/*
 * return - size of written data
 * */
#define sd_write_timeout(buff,size,time_ms) sd_write(buff,size,time_ms)

#define sd_syslock()
#define sd_sysunlock()
#define sd_wait_for_chars(s)

#define sd_protocol_inform(sequence,cmd,state) \
char *c; \
switch(state){ \
  case   SP_OK: \
    c="SP_OK"; \
  break; \
  case SP_UNCKNOWNCMD:\
    c="SP_UNCKNOWNCMD"; \
  break; \
  case SP_WRONGCHECKSUMM: \
    c="SP_WRONGCHECKSUMM"; \
  case SP_WRONGSIZE: \
    c="SP_WRONGSIZE"; \
  break; \
  default: \
    c="Uncknown state!"; \
} \
 \
printf("\r\ngot inform cmd(%d)[%d]=%d %s\r\n",(uint8_t)cmd,(uint8_t)sequence,(uint8_t)state,c)

#endif
