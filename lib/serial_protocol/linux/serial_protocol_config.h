#ifndef _SERIAL_PROTOCOL_CONFIG_H_
#define _SERIAL_PROTOCOL_CONFIG_H_


//#define _SERIAL_PROTOCOL_CHIBIOS_RT_

//#include "ch.h"
//#include "hal.h"
#include <time.h>
#include <pthread.h>
#include <errno.h>

# define timeradd(a, b, result) \
  do { \
    (result)->tv_sec = (a)->tv_sec + (b)->tv_sec; \
    (result)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec; \
    if ((result)->tv_nsec >= 1000000000) \
      { \
        ++(result)->tv_sec; \
        (result)->tv_nsec -= 1000000000; \
      } \
  } while (0)

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
  case SP_VERSION: \
    c="SP_VERSION"; \
  break; \
  default: \
    c="Uncknown state!"; \
} \
 \
printf("\r\ngot inform cmd(%d)[%d]=%d %s\r\n",(uint8_t)cmd,(uint8_t)sequence,(uint8_t)state,c)

extern pthread_mutex_t mutex;
extern pthread_mutex_t mutex_buffer;
extern uint32_t sd_last_system_message;
extern pthread_cond_t condition;

static inline int32_t sd_wait_system_message(uint8_t sequence, uint8_t cmd){
  int               rc;
  struct timespec   timeout,now,dt;
  dt.tv_sec = 0;
  dt.tv_nsec = 1000000*500;//ms

  //~ pthread_condattr_setclock(&condattr, CLOCK_MONOTONIC);

  // pthread_cond_timedwait() uses CLOCK_REALTIME to evaluate its
  // timeout argument.
  clock_gettime(CLOCK_MONOTONIC, &now);
  //~ clock_gettime(CLOCK_REALTIME, &now);

  timeradd(&dt,&now,&timeout);

  rc = pthread_mutex_lock(&mutex);
  do{
    rc = pthread_cond_timedwait(&condition, &mutex, &timeout);
    if( rc == 0){
      if((sd_last_system_message >> 16 & 0x7F) == (sequence & ~0x80) &&
      (  (sd_last_system_message >> 8  & 0x7F) == (cmd & ~0x80)
         || ( (cmd & ~0x80) == 0 &&  ((sd_last_system_message & 0xFF) == 4) ) /*cmd is version checksumm if SP_VERSION*/
      )){

          int32_t system_message = (sd_last_system_message & (0x7F << 16 | 0xFF <<8 | 0xFF));//it's oksd_wait_system_message, because usfulldata only in 0x7f7fff
          sd_last_system_message = 0;

          rc = pthread_mutex_unlock(&mutex);

          return system_message;//Delivered successful
      }else{
        //timeout (rc == ETIMEDOUT || rc == EINVAL || rc == EPERM)
        rc = pthread_mutex_unlock(&mutex);
        return -2;//timeout
      }
    }else{
      clock_gettime(CLOCK_MONOTONIC, &now);
      if(now.tv_sec <= timeout.tv_sec  && now.tv_nsec < timeout.tv_nsec) continue;//not timeout
      rc = pthread_mutex_unlock(&mutex);
      return -3;//wrong cmd
    }
  }while(1);

  rc = pthread_mutex_unlock(&mutex);
  return -2;//timeout anyway
}

static inline void sd_broadcast_system_message(uint8_t sequence, uint8_t cmd,uint8_t state){
  int rc;
  printf("sd_broadcast_system_message\r\n");
  rc = pthread_mutex_lock(&mutex);//calling thread blocks until the mutex becomes available.
  sd_last_system_message = ((uint32_t)sequence<<16 |(uint32_t)cmd<<8 |state);
  pthread_cond_broadcast(&condition);
  pthread_mutex_unlock(&mutex);
}

static inline uint16_t sd_lock_buffer(uint32_t time_ms){
  int               rc;
  struct timespec   timeout,now,dt;
  dt.tv_sec = 0;
  dt.tv_nsec = 1000000*time_ms;//ms

  clock_gettime(CLOCK_MONOTONIC, &now);

  timeradd(&dt,&now,&timeout);

  rc = pthread_mutex_timedlock(&mutex_buffer,&timeout);

  return ( rc == 0);
}

static inline uint16_t sd_unlock_buffer(){
  pthread_mutex_unlock(&mutex_buffer);
  return 1;//always true
}

#endif
