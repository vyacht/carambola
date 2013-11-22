#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <linux/poll.h>
#include <stdint.h>

#define	TIMEOUT		( 3 * 1000 )	/* poll timeout in milliseconds */


// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02

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
	char *filename = "/dev/vyspi0.0";
	struct	pollfd	fds[ 1 ];
	int i;
	int ret;
	
	signal(SIGINT, intHandler);

	fd = open(filename, O_RDWR | O_CREAT | O_TRUNC, 0);
	fds[0].fd = fd;
	fds[0].events = POLLRDNORM;

	if(!fd) {
		printf("Couldn't open device\n");
		exit(1);
	}

	unsigned char buf[255];

	uint32_t pgn = 0;
	uint32_t spiIn = 0;
	uint32_t spiOut = 0;
	uint32_t spiOldOut = 0;
	uint32_t spiLength = 0;

	while(keepRunning) {

		ssize_t status;
		while(!(status = poll( fds, 1, TIMEOUT ))) {
			printf( "Nobody ready right now, must wait...\n" );
			if(!keepRunning) goto close;
		}

		//		printf("poll() status= %d revents %d\n", status, fds[0].revents);

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
			uint8_t pkgOrg  = (uint8_t)((buf[0] & 0xF0) >> 5);
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
	close(fd);

	return 0;
}
