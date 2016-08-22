#ifndef _SERIAL_PROTOCOL_CONFIG_H_
#define _SERIAL_PROTOCOL_CONFIG_H_


#define _SERIAL_PROTOCOL_CHIBIOS_RT_

#include "ch.h"
#include "hal.h"
/* sd_get_timeout(ms)
 * return >= 0 - value
 * return <  0 - error
 * */
#define sd_get_timeout(time_ms) sdGetTimeout(&SD1,MS2ST(time_ms))

/*
 * return 0 - SUCCSESS
 * */
#define sd_put_timeout(byte,time_ms) sdPutTimeout(&SD1,byte,MS2ST(time_ms))
/*
 * return - size of written data
 * */
#define sd_write_timeout(buff,size,time_ms) sdWriteTimeout(&SD1,buff,size,MS2ST(time_ms))

#define sd_syslock() chSysLock()
#define sd_sysunlock() chSysUnlock()

#define sd_wait_for_chars(count) \
      while(1){ \
        chSysLock(); \
        if(chIQGetFullI(&((&SD1)->iqueue)) > count) break; \
        chSysUnlock(); \
        chThdYield();/*as fast as possible*/ \
      } \
      chSysUnlock();\


#endif
