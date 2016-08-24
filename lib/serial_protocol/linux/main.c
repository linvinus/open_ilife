

#include <stdlib.h>
int TTY_fd;

#include "serial_protocol_modules.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <stdint.h>
#include <pthread.h>

#define DEBUG_PROCOTOL(...) fprintf (stderr, __VA_ARGS__)
//~ #define DEBUG_SERIAL(...) fprintf (stderr, __VA_ARGS__)
#define DEBUG_SERIAL(...)

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        //~ bzero(&tty, sizeof(tty));
        //~ tty.c_cflag = speed | CRTSCTS | CS8 | CLOCAL | CREAD;
        //~ tty.c_iflag = IGNPAR;
        //~ tty.c_oflag = 0;

        //~ /* set input mode (non-canonical, no echo,...) */
        //~ tty.c_lflag = 0;

        //~ tty.c_cc[VTIME]    = 1;   /* inter-character timer unused */
        //~ tty.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

        tcflush(fd, TCIFLUSH);

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

int sd_read_byte(int time_ms){
  uint8_t byte=0;

  struct timeval timeout;
  fd_set set;
  int rv;
  FD_ZERO(&set); /* clear the set */
  FD_SET(TTY_fd, &set); /* add our file descriptor to the set */
  timeout.tv_sec = 0;
  timeout.tv_usec = 1000*time_ms;
  rv = select(TTY_fd + 1, &set, NULL, NULL, &timeout);
  if(rv == -1){
    DEBUG_SERIAL("select error\r\n"); /* an error accured */
    return -1;
  }else if(rv == 0){
    DEBUG_SERIAL("sd_read_byte timeout \r\n"); /* a timeout occured */
    return -1;
  }else{
     /* there was data to read */
    if(read( TTY_fd, &byte, 1 ) >= 0){
      DEBUG_SERIAL("r=0x%02x ",byte);
      return byte;
    }else{
      DEBUG_SERIAL("r=timeout ");
      return -1;
    }
  }
}

int sd_write_byte(char b,int time_ms){


  DEBUG_SERIAL("*** w=0x%02d ",(uint8_t)b);



  //~ struct timeval timeout;
  //~ fd_set set;
  //~ int rv;
  //~ FD_ZERO(&set); /* clear the set */
  //~ FD_SET(TTY_fd, &set); /* add our file descriptor to the set */
  //~ timeout.tv_sec = 0;
  //~ timeout.tv_usec = 1000*time_ms;
  //~ rv = select(TTY_fd + 1, &set, NULL, NULL, &timeout);
  //~ if(rv == -1){
    //~ perror("select error\r\n"); /* an error accured */
    //~ return -1;
  //~ }else if(rv == 0){
    //~ printf("sd_write_byte timeout \r\n"); /* a timeout occured */
    //~ return -1;
  //~ }else{
     /* there was data to read */
     int e = write(TTY_fd, &b, 1 );
     tcflush(TTY_fd, TCSADRAIN);
    return ( e == 1 ?  0 : -1);
  //~ }
}

int sd_write(uint8_t *buff,int size,int time_ms){
  int i;
  for(i=0;i<size;i++){
    DEBUG_SERIAL("w[%d]=0x%02x ",i,buff[i]);
  }
  //~ struct timeval timeout;
  //~ char buff[1];
  //~ fd_set set;
  //~ int rv;
  //~ FD_ZERO(&set); /* clear the set */
  //~ FD_SET(TTY_fd, &set); /* add our file descriptor to the set */
  //~ timeout.tv_sec = 0;
  //~ timeout.tv_usec = 1000*time_ms;
  //~ rv = select(TTY_fd + 1, &set, NULL, NULL, &timeout);
  //~ if(rv == -1){
    //~ perror("select error\r\n"); /* an error accured */
    //~ return -1;
  //~ }else if(rv == 0){
    //~ printf("sd_write timeout \r\n"); /* a timeout occured */
    //~ return -1;
  //~ }else{

      //return
      i = write(TTY_fd, buff, size );
      tcflush(TTY_fd, TCSADRAIN);
      return i;
  //~ }
}

int inputAvailable()
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  int rv = select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  if(rv ==-1 || rv == 0) return 0;
  else return 1;
  //~ return (FD_ISSET(STDIN_FILENO, &fds));
}

void serial_protocol_thread_fn(void){
  while(1){
    serial_protocol_main_loop_iterate();
    //~ usleep (1 * 1000); timeout in sd_read_byte is enough
  }
}
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_buffer = PTHREAD_MUTEX_INITIALIZER;

static pthread_condattr_t condattr;
pthread_cond_t condition = PTHREAD_COND_INITIALIZER;
uint32_t sd_last_system_message = 0;

int main(void) {

  pthread_t serial_protocol_thread;

  pthread_condattr_init(&condattr);
  pthread_condattr_setclock(&condattr, CLOCK_MONOTONIC);
  pthread_cond_init(&condition, &condattr);

  int  iret1;

  setbuf(stdout, NULL);// disable buffering entirely

  char *portname = "/dev/ttyUSB0";
  printf("opeinig port %s\r\n",portname);



  TTY_fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC );//O_NONBLOCK
  if (TTY_fd < 0)
  {
          printf ("error %d opening %s: %s", errno, portname, strerror (errno));
          return;
  }
  printf("TTY_fd=%d\r\n",TTY_fd);

  set_interface_attribs (TTY_fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (TTY_fd, 1);                // set no blocking
  tcflush(TTY_fd, TCIOFLUSH);

  //~ setbuf(TTY_fd, NULL);// disable buffering entirely
  //write (TTY_fd, "hello!\n", 7);           // send 7 character greeting

  //usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
                                     // receive 25:  approx 100 uS per char transmit
  //char buf [100];
  //int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

  iret1 = pthread_create( &serial_protocol_thread, NULL, serial_protocol_thread_fn, (void*) NULL);
  if(iret1){
    fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
    exit(EXIT_FAILURE);
  }
  int res=0;

  //check version on first connect
  int32_t remote_checksumm;
  do{
    remote_checksumm = serial_protocol_get_cmds_version();
  }while(remote_checksumm < 0);

  if( remote_checksumm >= 0){
    int32_t local_checksumm = calculate_version_checksumm();
    if( remote_checksumm == local_checksumm){
      printf("Version OK\r\n");
    }else{
      printf("ERROR: Version missmatch! %d != %d\r\n",local_checksumm,remote_checksumm);
      exit(1);
    }
  }

  struct timespec   start,end;

  while(1){
    //~ printf("\r\n###");
    //~ if(inputAvailable())
      //~ serial_protocol_get_cmd(1);

    //~ printf("###\r\n");
    //~ usleep (500 * 1000);
    printf("\r\n@@@");
    clock_gettime(CLOCK_MONOTONIC, &start);
    int res = serial_protocol_set_cmd(1,1);
    clock_gettime(CLOCK_MONOTONIC, &end);

    printf("@@@ %d s=%d %dms \r\n",res,end.tv_sec - start.tv_sec, (end.tv_nsec - start.tv_nsec)/1000) ;
    usleep (500 * 1000);
    //~ c = getch();
  }

}
