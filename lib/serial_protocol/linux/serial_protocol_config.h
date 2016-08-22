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

#endif
