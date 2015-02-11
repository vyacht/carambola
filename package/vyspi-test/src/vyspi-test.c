#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/poll.h>
#include <stdint.h>

#include <termios.h>
#include <unistd.h>
#include <errno.h>

#define	TIMEOUT		( 3 * 1000 )	/* poll timeout in milliseconds */

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02

#define VYSPI_RESET  0x04
#define VYSPI_PARAM  0x80

#define VYSPI_PARAM_CHANGE_SPEED 0x01 // change speed of a (uart) device

#define VYSPI_IOC_CMD_DATA_SIZE 10

struct vyspi_ioc_cmd_t {

  uint8_t      type;   // change speed, etc.
  uint8_t      data[VYSPI_IOC_CMD_DATA_SIZE]; // data

};

static int keepRunning = 1;

void intHandler(int dummy) {
    printf("Received sighandler\n");
    keepRunning = 0;
}

void get_val(uint8_t * buf, int offset, uint32_t * val) {
	*val = ((uint8_t)buf[offset]) << 24
        	        | ((uint8_t)buf[offset+1]) << 16
                	| ((uint8_t)buf[offset+2]) << 8
	                | ((uint8_t)buf[offset+3]);
}

void lookStats(uint8_t * buf) {

	        uint32_t pgn = ((int)buf[0]) << 24
        	        | ((int)buf[1]) << 16
                	| ((int)buf[2]) << 8
	                | ((int)buf[3]);
        	uint8_t packet_len = *(uint8_t *)&buf[4];

	        uint32_t frame_count = ((int)buf[5]) << 24
        	        | ((int)buf[6]) << 16
                	| ((int)buf[7]) << 8
	                | ((int)buf[8]);
	        uint32_t packet_count = ((int)buf[9]) << 24
        	        | ((int)buf[10]) << 16
                	| ((int)buf[11]) << 8
	                | ((int)buf[12]);
	        uint32_t packet_transfer_count = ((int)buf[13]) << 24
        	        | ((int)buf[14]) << 16
                	| ((int)buf[15]) << 8
	                | ((int)buf[16]);
	        uint32_t reset_count = ((int)buf[17]) << 24
        	        | ((int)buf[18]) << 16
                	| ((int)buf[19]) << 8
	                | ((int)buf[20]);
}

/* Function to get parity of number n. It returns 1
   if n has odd parity, and returns 0 if n has even
   parity */
int getParity(unsigned int n)
{
    int parity = 0;
    while (n)
    {
        parity = !parity;
        n      = n & (n - 1);
    }
    return parity;
}

typedef void(* st_decoder)(uint8_t * cmdBuffer, uint8_t size);

void processSTDepth(uint8_t * cmdBuffer, uint8_t size) {
  /*
    00  02  YZ  XX XX  Depth below transducer: XXXX/10 feet
                      Flags in Y: Y&8 = 8: Anchor Alarm is active
                                  Y&4 = 4: Metric display units or
                                           Fathom display units if followed by command 65
                                  Y&2 = 2: Used, unknown meaning
                      Flags in Z: Z&4 = 4: Transducer defective
                                  Z&2 = 2: Deep Alarm is active
                                  Z&1 = 1: Shallow Depth Alarm is active
                    Corresponding NMEA sentences: DPT, DBT
  */

  if(size == 5) {

    if(cmdBuffer[1] & 0x0f != 2) {
      printf("process error - wrong encoded data length\n");
    }

    if(cmdBuffer[2] & 0x80) {
      printf("Anchor alarm active\n");
    }
    if(cmdBuffer[2] & 0x40) {
      printf("Metric units for depth\n");
    }

    int depth = ((int)cmdBuffer[4] << 8) 
      | (int)cmdBuffer[3]; 

    printf("depth = %f feet\n", depth/10.0);

  } else {
    printf("process error - wrong datagram length\n");
  }
}

void processSTSpeed(uint8_t * cmdBuffer, uint8_t size) {
  float speed = ((int)cmdBuffer[3] << 8) | ((int)cmdBuffer[2]); 
  printf("speed = %f knots\n", speed/10.0);
}

void processSeatalkCmd(uint8_t * cmdBuffer, uint8_t size) {

  int i = 0;

    static struct
    {
	uint8_t cmdId;
	st_decoder decoder;
    } st_phrase[] = {
	/*@ -nullassign @*/
	{0x00, processSTDepth},	
	{0x20, processSTSpeed}
    };
    for (i = 0;
	 i < (unsigned)(sizeof(st_phrase) / sizeof(st_phrase[0])); ++i) {
      if(st_phrase[i].cmdId == cmdBuffer[0]) {
	st_phrase[i].decoder(cmdBuffer, size);
      }
    }

}

void printSeatalkChar(uint8_t c, int parerr, int * cmdFlag) {

  /* generally per definition: 
     - even parity bit set (1, high) if count of 1s is odd
     - even parity bit set (0, low) if count of 1s is even
  
  // 9th bit for command flag is interpreted as parity bit here
     we check for even parity

  // if the command flag is set then that means that parity even bit is set in the data stream

     
  */

  int parity = getParity(c); // 1 == odd, 0 == even

  if(!parity) {
    // input char has even parity
    if(parerr) {
      // and the parity error is signaled 
      // which means parity bit is 1
      *cmdFlag = 1;
       
    } else {
      // and no parity error is signaled
      // which means that the parity bit is not set
      *cmdFlag = 0;
    }

  } else {
    // input has odd parity
    if(parerr) {
      // and parity error is signaled
      // -> parity bit 0
      *cmdFlag = 0;
    } else {
      *cmdFlag = 1;
    }
  }

  // printf("%02X %s  ", c, cmdFlag?"(C)":"");

}

int seatalk(char * filename) {

  struct termios tty;
  struct termios tty_old;

  memset (&tty, 0, sizeof tty);

  printf("Opening device %s\n", filename);

  int USB = open( filename, O_RDONLY | O_NOCTTY );

  /* Error Handling */
  if ( tcgetattr ( USB, &tty ) != 0 )
  {
    printf("Error %d from tcgetattr %s\n", errno, strerror(errno));
  }

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B4800);
  cfsetispeed (&tty, (speed_t)B4800);

  /* Setting other Port Stuff */

  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CSTOPB;       // 1 stop

  tty.c_cflag     &=  ~CRTSCTS;       // no flow control
  tty.c_cflag     |= CLOCAL;     // 

  tty.c_cflag     |= CREAD;     // turn on READ

  tty.c_cflag     |=  PARENB;        // Make 8n1
  tty.c_cflag     &=  ~PARODD;        // Make 8n1

  tty.c_iflag     &=  ~IGNBRK;
  tty.c_iflag     |=  BRKINT;

  tty.c_iflag     &=  ~IGNPAR;
  tty.c_iflag     |=  PARMRK;
  tty.c_iflag     |=  INPCK;
  tty.c_iflag     &=  ~ISTRIP;

  tty.c_lflag     &=  ~ICANON;       // non canonical, we want every char directly

  tty.c_cc[VMIN]      =   1;                  // read doesn't block (non canonical)
  tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

  // Flush Port, then applies attributes 
  tcflush( USB, TCIFLUSH );

  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
    printf("Error %d from tcsetattr %s\n", errno, strerror(errno));
  }


#define PARITY_WARN 1
#define PARITY_ERR  2
#define STD_CHAR    3

  int n = 0, cnt = 0, i = 0;
  uint8_t buf;
  int state = STD_CHAR;

  uint8_t cmdBuf[255];
  int cmdCnt;

  do
  {
    cnt++;
    n = read( USB, &buf, 1 );
/*
    if((buf == 0x20) || (buf == 0x00)) 
	printf("\n");
    if(buf == 0xff)
	printf("#");
*/

    int cmdFlag = 0;
    int parErr = 0;

    printf(" %02X ", buf);

    switch(state) {

	case STD_CHAR:
	  if(buf == 0xff) {
            //printf("?", buf);
            state = PARITY_WARN;
	  } else {
            //printf(" %02X ", buf);
          }
	  break;

	case PARITY_ERR:
          //printf("e%02X ", buf);
          parErr = 1;
          state = STD_CHAR;
  	  break;

	case PARITY_WARN:
	  if(buf == 0xff) {
	    // relax, just a value 0xff
            state = STD_CHAR;
            //printf(" %02X ", buf);
          } else if(buf == 0x00) {
            state = PARITY_ERR;
            //printf("!", buf);
          }

	  break;

	default:
            printf("S%02X ", buf);
          break;
    };

    if(state == STD_CHAR) {
      printSeatalkChar(buf, parErr, &cmdFlag);
      if(cmdFlag) {
	if(cmdCnt > 0) {
	  printf("\n > ");
	  // new command just started, print and process the previous
	  for(i = 0; i < cmdCnt; i++) {
	    printf("%02X ", cmdBuf[i]);
	  }
	  printf("\n");
	  processSeatalkCmd(cmdBuf, cmdCnt);
	  cmdCnt = 0;
	}
      }
      cmdBuf[cmdCnt++] = buf;
    }
    fflush(stdout);
  }
  while((n > 0) && keepRunning);

  /*
  if ( tcsetattr ( USB, TCSANOW, &tty_old ) != 0) {
    printf("Error %d from restoring attributes %s\n", errno, strerror(errno));
  }
  */
  close(USB);
  printf("Exit after close\n");

  return 0;
}

int main(int argc, char * argv[]) {

  int fd;
  char *filename = NULL;
  struct	pollfd	fds[ 1 ];
  int i;
  int ret;

  int newSpeed = -1;
  int newOrigin = -1;
  int block = 0;
  int st = 0;
  int index;
  int c;
     
  opterr = 0;
     
  while ((c = getopt (argc, argv, "tbo:s:")) != -1) {
    switch (c)
    {
      case 't':
	st = 1;
	break;
      case 'b':
	block = 1;
	break;
      case 'o':
	newOrigin = atoi(optarg);
	break;
      case 's':
	newSpeed = atoi(optarg);
	break;
      case '?':
	if (optopt == 'c')
	  fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	else if (isprint (optopt))
	  fprintf (stderr, "Unknown option `-%c'.\n", optopt);
	else
	  fprintf (stderr,
		   "Unknown option character `\\x%x'.\n",
		   optopt);
	return 1;
      default:
	abort ();
      }
  }

  index = optind;
  if(index + 1 == argc) {
    // no non-option argument
    filename = malloc(strlen(argv[index] + 1));
    strcpy(filename, argv[index]);
  } else {
    filename = malloc(strlen("/dev/vyspi0.0") + 1);
    strcpy(filename, "/dev/vyspi0.0");
  }

  signal(SIGINT, intHandler);

  if(st) {
    return seatalk(filename);
  }

  printf("Opening device %s\n", filename);

  fd = open(filename, O_RDWR, 0);
  fds[0].fd = fd;
  fds[0].events = POLLRDNORM;

  if(!fd) {
    printf("Couldn't open device\n");
    exit(1);
  }

  if(block) {
    printf("Going to block on open mode\n");
    while(keepRunning) {
      printf("Keep on blocking\n");
      usleep(1000000 - 1);
    }
    close(fd);
    exit(0);
    printf("This should never be reached\n");
  }

  if((newOrigin > -1) && (newSpeed > -1)) {

    printf("Sleeping one sec\n");
    //    usleep(1000000 - 1);
    printf("Setting speed= %d for device= %d\n", newSpeed, newOrigin);
    struct vyspi_ioc_cmd_t cmd;
    memset(cmd.data, 0, VYSPI_IOC_CMD_DATA_SIZE);
    cmd.type = VYSPI_PARAM_CHANGE_SPEED;

    cmd.data[0] = newOrigin;
    cmd.data[1] = (newSpeed >> 24) & 0xFF; 
    cmd.data[2] = (newSpeed >> 16) & 0xFF; 
    cmd.data[3] = (newSpeed >> 8) & 0xFF; 
    cmd.data[4] = newSpeed & 0xFF;
    ioctl(fd, VYSPI_PARAM, &cmd);
    close(fd);
    exit(0);
  }

  printf("Opened device %s for reading\n", filename);

  unsigned char buf[255];

  uint32_t pgn = 0;
  uint32_t spiIn = 0;
  uint32_t spiOut = 0;
  uint32_t spiOldOut = 0;
  uint32_t spiLength = 0;

  ioctl(fd, VYSPI_RESET, NULL);
  usleep(1000000 - 1);

  while(keepRunning) {

    // printf( "Entering poll ...\n" );
    ssize_t status =  poll( fds, 1, TIMEOUT );
    while(!status) {
      printf( "Nobody ready right now, must wait...\n" );
      if(!keepRunning) goto close;
      //      printf( "Entering poll ...\n" );
      status =  poll( fds, 1, TIMEOUT );
    }
    // printf("poll() status= %d revents %d\n", status, fds[0].revents);

    int seenUnkown = 0;
    ssize_t r = read(fd, buf, 255);
    if(r > 0) {

      uint8_t packet_len = r;
      /*
	get_val(buf, 1, &spiIn);
	get_val(buf, 5, &spiOut);
	get_val(buf, 9, &spiLength);
	printf("in= %d, out=%d, length=%d, data len= %d, data= ", 
	spiIn, spiOut, spiLength, packet_len);
      */
      uint8_t pkgType = (uint8_t)buf[0] & 0x0F;
      uint8_t pkgOrg  = (uint8_t)((buf[0] & 0xF0) >> 4);
      uint8_t pkgLen =  (uint8_t)buf[1] & 0xFF;

      uint8_t len = 0;

      printf("total len = %d\n", packet_len); 

      while(len < packet_len) {

	if(pkgType == PKG_TYPE_NMEA2000) {

	  uint32_t pgn = 
	    ((uint32_t)buf[2]) << 24
	    | ((uint32_t)buf[3]) << 16
	    | ((uint32_t)buf[4]) << 8
	    | ((uint32_t)buf[5]);

	  uint32_t pkgid = 
	    ((uint32_t)buf[6]) << 24
	    | ((uint32_t)buf[7]) << 16
	    | ((uint32_t)buf[8]) << 8
	    | ((uint32_t)buf[9]);

	  printf("NMEA 2000: pid = %lu ", pkgid); 
	  printf("pgn = %lu ", pgn); 
	  printf("org = %lu ", pkgOrg); 
	  printf("len = %d", pkgLen); 

	  len += pkgLen + 8 + 2;

	  printf("\n");

	} else if (pkgType == PKG_TYPE_NMEA0183) {

	  printf("NMEA 0183: org = %lu ", pkgOrg); 
	  printf("len = %d: ", pkgLen); 
	  for (ret = 2; ret < pkgLen + 2; ret++) {
	    printf("%c", buf[ret] < 127 && buf[ret] > 31 ? buf[ret] : '.');
	  }
	  printf("\n");

	  len += pkgLen + 2;

	} else {
	  printf("UNKOWN %02X: org = %lu ", pkgType, 
		 pkgOrg); 
	  printf("len = %d", pkgLen); 
	  printf("\n");
	  seenUnkown = 1;
	  break;
	}

	if(pkgLen < 8) seenUnkown = 1;

      }

      // if(seenUnkown) 
      {

	ret = 0;
        // header first
	while((ret < packet_len) && (ret < 10))  {
	  printf("0x%02X ", (uint8_t)buf[ret]); ret++;
	}
	printf(" : ");
	ret = 0;
	while((ret < packet_len) && (ret < 10))  {
	  printf("%c", buf[ret] < 127 && buf[ret] > 31 ? buf[ret] : '.');
	  ret++;
	}
	printf("\n");

	ret = 0;
	while(ret < packet_len)  {
	  int i = 0;
	  for (i = 0; i < 8; i++) {
	    if(ret + i < packet_len)
	      printf("0x%02X ", (uint8_t)buf[ret + i]);
	    else
	      printf("     ");
	  }
	  printf(" : ");
	  for (i = 0; i < 8; i++) {
	    if(ret + i < packet_len)
	      printf("%c", buf[ret + i] < 127 && buf[ret + i] > 31 ? buf[ret + i] : '.');
	    else
	      printf(" ");
	  }
	  ret += 8;
	  printf("\n");
	}
	printf("\n");
	if (seenUnkown)
	  goto close;
      }
			
    } else {
      printf("Zero length read: %d\n", r); 
    }
  }	
 close:
  printf("Closing device %s\n", filename);
  close(fd);

  return 0;
}
