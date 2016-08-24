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

#define sd_protocol_inform(sequence,cmd,state)

static threads_queue_t       sd_protocol_q_waiting;

static BSEMAPHORE_DECL(SD_BUFF_SEM,FALSE);

static inline int32_t sd_wait_system_message(uint8_t sequence, uint8_t cmd){
  uint32_t start = chTimeNow();
  uint32_t elapsed = 0;
  do{
    msg_t msg = chThdEnqueueTimeoutS(&sd_protocol_q_waiting,MS2ST(500) - elapsed);
    if(msg == MSG_TIMEOUT){
      return -2;//timeout
    }else if( (msg >> 16 & 0x7F) == (sequence & ~0x80) &&
              (msg >> 8 & 0x7F) == (cmd & ~0x80) ){
          int32_t system_message = msg & (0x7F << 16 | 0x7F <<8 | 0xFF);//it's oksd_wait_system_message, because usfulldata only in 0x7f7fff
          return system_message;//Delivered successful
    }
  }while( (elapsed = chTimeElapsedSince(start)) < MS2ST(500) );//osalOsIsTimeWithinX ?
  return -2;//timeout anyway
}

static inline void sd_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state){
  chThdDequeueAllI(&sd_protocol_q_waiting,((uint32_t)sequence<<16 |(uint32_t)cmd<<8 |state));
}

static inline uint16_t sd_lock_buffer(uint32_t time_ms){
  return (chBSemWaitTimeout(&SD_BUFF_SEM,MS2ST(time_ms)) == MSG_OK);
}

static inline uint16_t sd_unlock_buffer(){
  chBSemSignal(&SD_BUFF_SEM);
  return 1;//always true
}

#endif
