
#include "serial_protocol.h"
#include "cobs.h"

#define BUFFER_LENGTH 255
#define COBSBUF_RAW_DATA_OFFCET 4

#define SD_HEADER_SIZE 4
#define SD_MAX_PACKET (BUFFER_LENGTH - 1 - SD_HEADER_SIZE - COBSBUF_RAW_DATA_OFFCET) /*246 bytes*/

static uint8_t cobs_buf1[BUFFER_LENGTH];

static uint8_t * const cobs_buf_p = (uint8_t *)&cobs_buf1;

static uint8_t last_sequence;

typedef struct{
 uint8_t sequence;  // 7bit, 8th bit (0x80) indicate that comfirm  is requested
 uint8_t cmd;       // 7bit, 8th bit indicate GET (0x80) or SET command
 uint8_t size;      // body size
 uint8_t invchksumm;// body inverse check summ
}sd_header_t;

typedef enum {
  SP_SYSTEM_MESSAGE = 0,
  SP_CONFIGURATION = 1
}SerialPacketType_t;

/*
 * Because packet whole encoded as header + body,
 * but we parse separately firstly head then body,
 * we need store cobs-decoder state between calls
 * */

typedef struct{
  int32_t n;    //current processed byte
  int32_t code; //last code
} cobs_state_t;

static cobs_state_t cobs_state_rx; //store state between calls of cobs_receive_decode

#define cobs_rx_reset() {cobs_state_rx.n = 0;}

static int16_t build_and_send_package(sd_header_t *hdr,size_t bodysize,uint8_t* body){
  uint16_t pktsize=0,i;
  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;//after cobs_encode data will be starting from [0]
  uint8_t *hdrp=(uint8_t *)hdr;

  raw[0]=hdrp[0];//sequence
  raw[1]=hdrp[1];//cmd


  if(bodysize>0){
    uint8_t *raw_body = raw + SD_HEADER_SIZE;
    uint8_t raw_body_checksumm = 0;
    //copy body to temporary buffer, calculate checksumm
    sd_syslock();
      for(i=0; i < bodysize; i++){
        raw_body[i] = body[i];
        raw_body_checksumm += body[i];
      }
      //~ memccpy(,,bodysize,sizeof(uint8_t));
    sd_sysunlock();

    raw[2] = bodysize;//size
    raw[3] = (uint8_t)~raw_body_checksumm;//invchksumm
  }else{
    raw[2] = hdrp[2];//in size, some usefull data
    raw[3] = hdrp[3];//in invchksumm, some usefull data
  }

  pktsize = bodysize + SD_HEADER_SIZE;//body + header

  pktsize = cobs_encode(raw,pktsize,cobs_buf_p);//move to the left

  if( sd_put_timeout(COBS_SYMBOL,100) == 0 ){//start of packet
    size_t rc = sd_write_timeout(cobs_buf_p,pktsize,500);//send encoded data
    return (rc == pktsize ? (int16_t)rc : -1 ); //-1 //protocol error
  }
  else return -1; //protocol error

}//build_and_send_package

static size_t cobs_receive_decode(size_t pktsize, uint8_t* destination,uint8_t invchksumm){

  uint8_t *dst = destination;
  uint8_t *end = dst + pktsize;
  uint8_t chksumm = 0;
  int32_t c;


  while(dst < end)
  {
      if(cobs_state_rx.n < 2){ //load new code only if previous was fully parsed (cobs_state_rx.n==1 || cobs_state_rx.n==0)

        cobs_state_rx.code = sd_get_timeout(100);

        if(cobs_state_rx.code < 0 || cobs_state_rx.code == COBS_SYMBOL /*|| ( (cobs_state_rx.read_index + cobs_state_rx.code) > end && cobs_state_rx.code != 1)*/)
            return 0;//error

        cobs_state_rx.n = cobs_state_rx.code;
      }

      while( cobs_state_rx.n > 1 ){
        if(dst < end){

          c = sd_get_timeout(100);

          if(c < 0 || c == COBS_SYMBOL)//COBS_SYMBOL is not allowed there!
            return 0;//error

          cobs_state_rx.n--;
          chksumm += *(dst++) = c;
        }else
          goto FINISH;//package was partially received, got requested pktsize
      }


      if(cobs_state_rx.code != 0xFF && dst != end)
      {
          chksumm += *(dst++) = COBS_SYMBOL;
          cobs_state_rx.n--;
      }
  }
  FINISH:

  //warning: comparison of promoted ~unsigned with unsigned https://gcc.gnu.org/bugzilla/show_bug.cgi?id=38341
  if( invchksumm !=0 && ((uint8_t)~chksumm) != invchksumm)
    return 0;//error, broken package, checksum mismatch

  return (dst - destination);
}//cobs_receive_decode

static inline size_t cobs_start_receiving_and_decode(size_t pktsize, uint8_t* destination){
  cobs_rx_reset();
  return cobs_receive_decode(pktsize, destination,0);
}


static inline void skip(sd_header_t *hdr){
  //skip current packet body
  uint8_t s = hdr->size;
  while( s-- > 0 && sd_get_timeout(100) >=0)
    ;
}

static inline void answer(sd_header_t *hdr, SerialPacketSystemMessage_t reason){
  hdr->sequence &= ~0x80;                        //remove confirm bit
  hdr->size = hdr->cmd;                          //store current cmd in size
  hdr->cmd = SP_SYSTEM_MESSAGE;
  hdr->invchksumm = reason;                      //reason
  build_and_send_package(hdr, 0, NULL);
}

static inline void skip_and_aswer(sd_header_t *hdr, SerialPacketSystemMessage_t err){
  skip(hdr);
  answer(hdr,err);
}

static uint8_t calculate_version_checksumm(void){
    uint8_t verchksumm,i;
    verchksumm = SD_MAX_PACKET ^ SD_CMDS_COUNT;
    for(i=0;i<SD_CMDS_COUNT;i++){
      verchksumm ^= (SD_CMDS[i].rx_data_size
                 ^   SD_CMDS[i].tx_data_size
                 ^ ( (SD_CMDS[i].rx_data == NULL)<<1
                   | (SD_CMDS[i].rx_callback == NULL)<<2
                   | (SD_CMDS[i].rx_arg == NULL)<<3
                   | (SD_CMDS[i].tx_data == NULL)<<4
                   | (SD_CMDS[i].tx_callback == NULL)<<5
                   | (SD_CMDS[i].tx_arg == NULL)<<6 ) );
    }
    return verchksumm;
}

static inline void _sd_main_loop_iterate(void){
    int32_t c = -1;
    c = sd_get_timeout(100);

    //~ if(c >=0) sd_put_timeout(c,100);//loopback

    if(c == COBS_SYMBOL){
      /* got packet delimeter,
       * recieve header
       * */
      uint8_t header[SD_HEADER_SIZE]={0};
      sd_header_t *hdr = (sd_header_t *)header;

      sd_wait_for_chars(3);//wait for count of SD_HEADER_SIZE

      if( cobs_start_receiving_and_decode(SD_HEADER_SIZE,header) == SD_HEADER_SIZE ){
        //got header
        if(hdr->cmd == (SP_SYSTEM_MESSAGE & 0x80)){//get version
          //cmd contain protocol checksumm
          hdr->cmd = calculate_version_checksumm();
          answer(hdr,SP_VERSION);
        }else if(hdr->cmd == SP_SYSTEM_MESSAGE){//received protocol information
            sd_broadcast_system_message(hdr->sequence,hdr->size,hdr->invchksumm);
            sd_protocol_inform(hdr->sequence,hdr->size,hdr->invchksumm);
        }else{
          last_sequence = hdr->sequence;

          if(hdr->size < SD_MAX_PACKET ){
            //hdr->cmd == SP_SYSTEM_MESSAGE
            if( (hdr->cmd & ~0x80) < SD_CMDS_COUNT ){//known CMD

              if(hdr->cmd & 0x80){
                //get CMD
                hdr->cmd &= ~0x80;//remove GET bit

                build_and_send_package(hdr,SD_CMDS[hdr->cmd].tx_data_size,SD_CMDS[hdr->cmd].tx_data);

              }else{//set CMD
                if( SD_CMDS[hdr->cmd].rx_data_size == hdr->size ){

                  //receive body in temporary buffer
                  if( cobs_receive_decode(hdr->size, cobs_buf_p, hdr->invchksumm) == hdr->size){

                    //got body
                    uint8_t* rx_data = SD_CMDS[hdr->cmd].rx_data;

                    //copy body to destination
                    sd_syslock();
                    uint16_t i;
                    for(i=0; i < hdr->size; i++){
                      rx_data[i]=cobs_buf_p[i];
                    }
                    sd_sysunlock();

                    //callback
                    if(SD_CMDS[hdr->cmd].rx_callback != NULL){
                      SD_CMDS[hdr->cmd].rx_callback(SD_CMDS[hdr->cmd].rx_arg);
                    }

                    if(hdr->sequence & 0x80){ //comfirm requested
                      //send ak
                      answer(hdr,SP_OK);
                    }
                  }else //body error
                    answer(hdr,SP_WRONGCHECKSUMM); //don't skip because already partially or completely received

                }else
                  skip_and_aswer(hdr,SP_WRONGSIZE);

              }//CMD set
            }else
              skip_and_aswer(hdr,SP_UNCKNOWNCMD);

          }else
            skip_and_aswer(hdr,SP_WRONGSIZE);
        }//nonsystem message

      }//if got header

    }//if COBS_SYMBOL
}//_sd_main_loop

#if defined(_SERIAL_PROTOCOL_CHIBIOS_RT_)
static THD_WORKING_AREA(waThreadSerialProtocol, 256);
static THD_FUNCTION(ThreadSerialProtocol, arg) {
  (void)arg;
  while(1){
    _sd_main_loop_iterate();
    chThdYield();
    //~ chThdSleepMilliseconds(1);
  }//while 1
}//ThreadSerialProtocol


void serial_protocol_thread_init(void){
  chThdQueueObjectInit(&sd_protocol_q_waiting);
  chThdCreateStatic(waThreadSerialProtocol, sizeof(waThreadSerialProtocol), NORMALPRIO+1, ThreadSerialProtocol, NULL);
}
#endif /*_SERIAL_PROTOCOL_CHIBIOS_RT_*/

void serial_protocol_main_loop_iterate(void){
  _sd_main_loop_iterate();
}

int serial_protocol_get_cmd(uint8_t cmd){
  if(cmd < SD_CMDS_COUNT )
  {
    sd_header_t hdr1;
    sd_header_t  *hdr = &hdr1;
    hdr->sequence = (++last_sequence) & ~0x80; //remove confirm bit, the answer is our confirm
    hdr->cmd = cmd | 0x80;//GET
    hdr->size = SD_CMDS[cmd].tx_data_size;
    hdr->invchksumm = 0;
    build_and_send_package(hdr, 0, NULL);
    return 0;//OK
  }else
    return -1;//ERROR
}

int serial_protocol_set_cmd(uint8_t cmd, uint8_t confirm){
  if(cmd < SD_CMDS_COUNT )
  {
    sd_header_t hdr1;
    sd_header_t  *hdr = &hdr1;
    hdr->sequence = (++last_sequence);
    hdr->sequence |= (confirm ?  0x80 : 0);
    hdr->cmd = cmd & ~0x80;//SET
    build_and_send_package(hdr,SD_CMDS[hdr->cmd].tx_data_size,SD_CMDS[hdr->cmd].tx_data);
    if(confirm){
      return sd_wait_system_message(hdr->sequence,hdr->cmd);//-2 - timeout, 0 - successful
    }
    return 0;//OK
  }else
    return -1;//ERROR
}
