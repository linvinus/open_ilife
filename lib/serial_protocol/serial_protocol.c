
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
  SP_OK,
  SP_UNCKNOWNCMD,
  SP_WRONGCHECKSUMM,
  SP_WRONGSIZE
}SerialPacketSystemMessage_t;

typedef enum {
  SP_SYSTEM_MESSAGE = 0,
  SP_CONFIGURATION = 1
}SerialPacketType_t;


static int16_t build_and_send_package(SerialDriver *sdp,sd_header_t *hdr,size_t bodysize,uint8_t* body){
  uint16_t pktsize=0,i;
  uint8_t *raw = cobs_buf_p + COBSBUF_RAW_DATA_OFFCET;
  uint8_t *hdrp=(uint8_t *)hdr;
  
  raw[0]=hdrp[0];//sequence
  raw[1]=hdrp[1];//cmd
  

  if(bodysize>0){
    uint8_t *raw_body = raw + SD_HEADER_SIZE;
    uint8_t raw_body_checksumm = 0;
    //copy body to temporary buffer, calculate checksumm
    chSysLock();  
      for(i=0; i < bodysize; i++){
        raw_body[i] = body[i];
        raw_body_checksumm += body[i];
      }
      //~ memccpy(,,bodysize,sizeof(uint8_t));
    chSysUnlock();

    raw[2] = bodysize;//size
    raw[3] = (uint8_t)~raw_body_checksumm;//invchksumm
  }else{
    raw[2] = hdrp[2];//in size, some usefull data
    raw[3] = hdrp[3];//in invchksumm, some usefull data
  }
  
  pktsize = bodysize + SD_HEADER_SIZE;//body + header
  
  cobs_encode(raw,pktsize,cobs_buf_p);//move to the left
  
  if( sdPutTimeout(sdp,COBS_SYMBOL,MS2ST(100)) == Q_OK ){//start of packet
    size_t rc = sdWriteTimeout(sdp,cobs_buf_p,pktsize,MS2ST(500));//send encoded data
    return (rc == pktsize ? (int16_t)rc : -1 ); //-1 //protocol error
  }
  else return -1; //protocol error
   
}//build_and_send_package

static size_t cobs_receive_decode(SerialDriver *sdp,size_t pktsize, uint8_t* destination,uint8_t invchksumm){
  int32_t code,code2;
  uint8_t* dst = destination;
  uint8_t* end = destination + pktsize;
  uint8_t chksumm = 0;


  while(dst < end)
  {
      code = sdGetTimeout(sdp,MS2ST(100));//read_index

      if(code < 0 || ( (dst + code) > end && code != 1))
      {
          return 0;
      }
      code2 = code - 1;
      //read_index++;

      //~ size_t rc = sdReadTimeout(sdp,dst,code2,MS2ST(500));
      //~ if(rc == code2){
        //~ dst         += code2;
        //~ read_index  += code2;
      //~ }else
        //~ return 0;//error
      uint16_t i;
      for(i=0; i < code2; i++){
        code = sdGetTimeout(sdp,MS2ST(100));//read_index
        if(code < 0) return 0;//error
        *(dst++) = code;
        chksumm += code;
      }


      if(code != 0xFF && dst != end)
      {
          *(dst++) = COBS_SYMBOL;
          chksumm += COBS_SYMBOL;
      }
  }
  //warning: comparison of promoted ~unsigned with unsigned https://gcc.gnu.org/bugzilla/show_bug.cgi?id=38341
  if( invchksumm !=0 && ((uint8_t)~chksumm) != invchksumm) return 0;//error

  return (dst - destination);
}//cobs_receive_decode

/*
static int32_t get_packet(SerialDriver *sdp, int32_t size, uint8_t* destination){
  uint16_t s = cobs_get_encoded_buffer_size(size);
  uint8_t buffer[s];
  size_t rc = sdReadTimeout(sdp,buffer,size,MS2ST(500));
  if(rc == size){
    cobs_decode(buffer,size,destination);
  }
  //else //protocol error
}
*/

/*
static int32_t cobs_encode_send(SerialDriver *sdp, int32_t pktsize, uint8_t* src){
  uint16_t s = cobs_get_encoded_buffer_size(pktsize)+1;
  //~ uint8_t buffer[s];
  cobs_buf_p[0]=COBS_SYMBOL;
  cobs_encode(src,pktsize,cobs_buf+1);
  ~body[hdr->size-1]
  size_t rc = sdWriteTimeout(sdp,buffer,s,MS2ST(500));

  return (rc == s ? rc : -1 ) //-1 //protocol error
}*/

static inline void skip(SerialDriver *sdp, sd_header_t *hdr){
  //skip current packet body
  uint8_t s = hdr->size;
  while( s-- > 0 )
    sdGetTimeout(sdp,MS2ST(100));  
}

static inline void answer(SerialDriver *sdp, sd_header_t *hdr, SerialPacketSystemMessage_t reason){
  hdr->sequence &= ~0x80;                        //remove confirm bit
  hdr->size = hdr->cmd;                          //store current cmd in size
  hdr->cmd = SP_SYSTEM_MESSAGE;
  hdr->invchksumm = reason;                      //reason
  build_and_send_package(sdp, hdr, 0, NULL);
}

static inline void skip_and_aswer(SerialDriver *sdp, sd_header_t *hdr, SerialPacketSystemMessage_t err){
  skip(sdp,hdr);
  answer(sdp,hdr,err);
}

static THD_WORKING_AREA(waThreadSerialProtocol, 256);
static THD_FUNCTION(ThreadSerialProtocol, arg) {
  //(void)arg;
  SerialDriver *sdp=(SerialDriver *)arg;
  while(1){
    int32_t c=0;
    c = sdGetTimeout(sdp,MS2ST(100));
    if(c == COBS_SYMBOL){
      /* got packet delimeter,
       * recieve header
       * */
      uint8_t header[SD_HEADER_SIZE];
      sd_header_t *hdr = (sd_header_t *)header;
      if( cobs_receive_decode(sdp,SD_HEADER_SIZE,header,0) == SD_HEADER_SIZE ){
        last_sequence = hdr->sequence;
        //got header
        if(hdr->size < SD_MAX_PACKET ){
          //hdr->cmd == SP_SYSTEM_MESSAGE
          if( (hdr->cmd & ~0x80) < SD_CMDS_COUNT ){//known CMD

            if(hdr->cmd & 0x80){
              //get CMD
              hdr->cmd &= ~0x80;//remove GET bit

              build_and_send_package(sdp,hdr,SD_CMDS[hdr->cmd].tx_data_size,SD_CMDS[hdr->cmd].tx_data);
              
            }else{//set CMD
              if( SD_CMDS[hdr->cmd].rx_data_size == hdr->size ){

                //receive body in temporary buffer
                if( cobs_receive_decode(sdp, hdr->size, cobs_buf_p, hdr->invchksumm) == hdr->size){

                  //got body
                  uint8_t* rx_data = SD_CMDS[hdr->cmd].rx_data;

                  //copy body to destination
                  chSysLock();
                  uint16_t i;
                  for(i=0; i < hdr->size; i++){
                    rx_data[i]=cobs_buf_p[i];
                  }
                  chSysUnlock();

                  //callback
                  if(SD_CMDS[hdr->cmd].rx_callback != NULL){
                    SD_CMDS[hdr->cmd].rx_callback(SD_CMDS[hdr->cmd].rx_arg);
                  }

                  if(hdr->sequence & 0x80){ //comfirm requested
                    //send ak                  
                    answer(sdp,hdr,SP_OK);
                  }
                }else //body error
                  answer(sdp,hdr,SP_WRONGCHECKSUMM); //don't skip because already partially or completely received
                
              }else
                skip_and_aswer(sdp,hdr,SP_WRONGSIZE);
              
            }//CMD set
          }else
            skip_and_aswer(sdp,hdr,SP_UNCKNOWNCMD);
          
        }else
          skip_and_aswer(sdp,hdr,SP_WRONGSIZE);
        
      }//if got header

      /*
      int32_t pktSize = sdGetTimeout(sdp,MS2ST(100));
      if(pktSize > 0 ){ //minimal packet size one byte, or more (cobs allow zero bytes)
        c = sdGetTimeout(sdp,MS2ST(100));//packet type
        if(c > 1 ){ //minimal packet size two bytes, or more (cobs allow zero bytes)

          if( (c & ~(0x80)) < sizeof(SD_CMDS)){
            if()
          }

          switch(c){
            SP_GET_CONFIGURATION:
              send_packet(sdp,sizeof(RobotCFG_t),&RobotCFG);
            break;
            default:
              switch(c | 128){
                SP_SET_CONFIGURATION:
                  get_packet(sdp,pktSize,&RobotCFG);
                break;
                default:
                //unknown!!!
                break;
              }
            break;
          }
        }
      }
      //else //protocol error
      */
       
    }//if COBS_SYMBOL
  }//while 1
}//ThreadSerialProtocol


void serial_protocol_init(SerialDriver *sdp){
  chThdCreateStatic(waThreadSerialProtocol, sizeof(waThreadSerialProtocol), NORMALPRIO+1, ThreadSerialProtocol, sdp);
}
