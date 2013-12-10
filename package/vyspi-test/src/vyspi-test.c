#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/poll.h>
#include <stdint.h>

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

int main(int argc, char * argv[]) {

  int fd;
  char *filename = NULL;
  struct	pollfd	fds[ 1 ];
  int i;
  int ret;

  int newSpeed = -1;
  int newOrigin = -1;
  int block = 0;
  int index;
  int c;
     
  opterr = 0;
     
  while ((c = getopt (argc, argv, "bo:s:")) != -1) {
    switch (c)
    {
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
    cmd.data[1] = (newSpeed >> 8) & 0xFF; 
    cmd.data[2] = newSpeed & 0xFF;
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

	if(pkgLen < 10) seenUnkown = 1;

      }

      if(seenUnkown) {

	ret = 0;
	while(ret < packet_len)  {
	  int i = 0;
	  for (i = 0; i < 8; i++) {
	    if(ret + i < packet_len)
	      printf("0x%02X ", (uint8_t)buf[ret + i]);
	    else
	      printf("0x00 ");
	  }
	  printf(" : ");
	  for (i = 0; i < 8; i++) {
	    if(ret + i < packet_len)
	      printf("%c", buf[ret + i] < 127 && buf[ret + i] > 31 ? buf[ret + i] : '.');
	    else
	      printf(".");
	  }
	  ret += 8;
	  printf("\n");
	}
	printf("\n");
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
