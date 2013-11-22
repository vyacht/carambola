/*
 * NMEA2000 over CAN.
 *
 * This file is Copyright (c) 2012 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#ifndef S_SPLINT_S
#include <unistd.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#endif /* S_SPLINT_S */

#include "gpsd.h"
#if defined(VYSPI_ENABLE)
#include "driver_vyspi.h"
#include "bits.h"

#define LOG_FILE 1

typedef struct PGN
    {
    unsigned int  pgn;
    unsigned int  fast;
    unsigned int  type;
    gps_mask_t    (* func)(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);
    const char    *name;
    } PGN;

/*@-nullassign@*/
static void print_data(unsigned char *buffer, int len, PGN *pgn)
{
#ifdef LIBGPS_DEBUG
    /*@-bufferoverflowhigh@*/
    if ((libgps_debuglevel >= LOG_IO) != 0) {
	int   l1, l2, ptr;
	char  bu[128];

        ptr = 0;
        l2 = sprintf(&bu[ptr], "got data:%6u:%3d: ", pgn->pgn, len);
	ptr += l2;
        for (l1=0;l1<len;l1++) {
            if (((l1 % 20) == 0) && (l1 != 0)) {
	        gpsd_report(LOG_IO,"%s\n", bu);
		ptr = 0;
                l2 = sprintf(&bu[ptr], "                   : ");
		ptr += l2;
            }
            l2 = sprintf(&bu[ptr], "%02ux ", (unsigned int)buffer[l1]);
	    ptr += l2;
        }
        gpsd_report(LOG_IO,"%s\n", bu);
    }
    /*@+bufferoverflowhigh@*/
#endif
}


static gps_mask_t get_mode(struct gps_device_t *session)
{
    if (session->driver.nmea2000.mode_valid) {
        session->newdata.mode = session->driver.nmea2000.mode;
    } else {
        session->newdata.mode = MODE_NOT_SEEN;
    }

    return MODE_SET;
}


static gps_mask_t hnd_059392(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_060928(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_126208(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_126464(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_126996(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129025(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    /*@-type@*//* splint has a bug here */
    session->newdata.latitude = getles32(bu, 0) * 1e-7;
    session->newdata.longitude = getles32(bu, 4) * 1e-7;
    /*@+type@*/

    (void)strlcpy(session->gpsdata.tag, "129025", sizeof(session->gpsdata.tag));

    return LATLON_SET | get_mode(session);
}


static gps_mask_t hnd_129026(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    session->driver.nmea2000.sid[0]  =  bu[0];

    /*@-type@*//* splint has a bug here */
    session->newdata.track           =  getleu16(bu, 2) * 1e-4 * RAD_2_DEG;
    session->newdata.speed           =  getleu16(bu, 4) * 1e-2;
    /*@+type@*/

    (void)strlcpy(session->gpsdata.tag, "129026", sizeof(session->gpsdata.tag));

    return SPEED_SET | TRACK_SET | get_mode(session);
}


static gps_mask_t hnd_126992(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    //uint8_t        sid;
    //uint8_t        source;

    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    //sid        = bu[0];
    //source     = bu[1] & 0x0f;

    /*@-type@*//* splint has a bug here */
    session->newdata.time = getleu16(bu, 2)*24*60*60 + getleu32(bu, 4)/1e4;
    /*@+type@*/

    (void)strlcpy(session->gpsdata.tag, "126992", sizeof(session->gpsdata.tag));

    return TIME_SET | get_mode(session);
}


static const int mode_tab[] = {MODE_NO_FIX, MODE_2D,  MODE_3D, MODE_NO_FIX,
			       MODE_NO_FIX, MODE_NO_FIX, MODE_NO_FIX, MODE_NO_FIX};

static gps_mask_t hnd_129539(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    gps_mask_t mask;

    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    mask                             = 0;
    session->driver.nmea2000.sid[1]  = bu[0];

    session->driver.nmea2000.mode_valid = 1;

    session->driver.nmea2000.mode    = mode_tab[(bu[1] >> 3) & 0x07];

    /*@-type@*//* splint has a bug here */
    session->gpsdata.dop.hdop        = getleu16(bu, 2) * 1e-2;
    session->gpsdata.dop.vdop        = getleu16(bu, 4) * 1e-2;
    session->gpsdata.dop.tdop        = getleu16(bu, 6) * 1e-2;
    /*@+type@*/
    mask                            |= DOP_SET;

    gpsd_report(LOG_DATA, "pgn %6d(%3d): sid:%02x hdop:%5.2f vdop:%5.2f tdop:%5.2f\n",
		pgn->pgn,
		session->driver.nmea2000.unit,
		session->driver.nmea2000.sid[1],
		session->gpsdata.dop.hdop,
		session->gpsdata.dop.vdop,
		session->gpsdata.dop.tdop);

    (void)strlcpy(session->gpsdata.tag, "129539", sizeof(session->gpsdata.tag));

    return mask | get_mode(session);
}


static gps_mask_t hnd_129540(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    int         l1, l2;

    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    session->driver.nmea2000.sid[2]           = bu[0];
    session->gpsdata.satellites_visible       = (int)bu[2];

    for (l2=0;l2<MAXCHANNELS;l2++) {
        session->gpsdata.used[l2] = 0;
    }
    l2 = 0;
    for (l1=0;l1<session->gpsdata.satellites_visible;l1++) {
        int    svt;
        double azi, elev, snr;

	/*@-type@*//* splint has a bug here */
        elev  = getles16(bu, 3+12*l1+1) * 1e-4 * RAD_2_DEG;
        azi   = getleu16(bu, 3+12*l1+3) * 1e-4 * RAD_2_DEG;
        snr   = getles16(bu, 3+12*l1+5) * 1e-2;
	/*@+type@*/

        svt   = (int)(bu[3+12*l1+11] & 0x0f);

        session->gpsdata.elevation[l1]  = (int) (round(elev));
	session->gpsdata.azimuth[l1]    = (int) (round(azi));
        session->gpsdata.ss[l1]         = snr;
        session->gpsdata.PRN[l1]        = (int)bu[3+12*l1+0];
	if ((svt == 2) || (svt == 5)) {
	    session->gpsdata.used[l2] = session->gpsdata.PRN[l1];
	    l2 += 1;
	}
    }
    return  SATELLITE_SET | USED_IS;
}


static gps_mask_t hnd_129029(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    gps_mask_t mask;

    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);

    mask                             = 0;
    session->driver.nmea2000.sid[3]  = bu[0];
 
    /*@-type@*//* splint has a bug here */
    session->newdata.time            = getleu16(bu,1) * 24*60*60 + getleu32(bu, 3)/1e4;
    /*@+type@*/
    mask                            |= TIME_SET;

    /*@-type@*//* splint has a bug here */
    session->newdata.latitude        = getles64(bu, 7) * 1e-16;
    session->newdata.longitude       = getles64(bu, 15) * 1e-16;
    /*@+type@*/
    mask                            |= LATLON_SET;

    /*@-type@*//* splint has a bug here */
    session->newdata.altitude        = getles64(bu, 23) * 1e-6;
    /*@+type@*/
    mask                            |= ALTITUDE_SET;

//  printf("mode %x %x\n", (bu[31] >> 4) & 0x0f, bu[31]);
    switch ((bu[31] >> 4) & 0x0f) {
    case 0:
        session->gpsdata.status      = STATUS_NO_FIX;
	break;
    case 1:
        session->gpsdata.status      = STATUS_FIX;
	break;
    case 2:
        session->gpsdata.status      = STATUS_DGPS_FIX;
	break;
    case 3:
    case 4:
    case 5:
        session->gpsdata.status      = STATUS_FIX; /* Is this correct ? */
	break;
    default:
        session->gpsdata.status      = STATUS_NO_FIX;
	break;
    }
    mask                            |= STATUS_SET;

    /*@-type@*//* splint has a bug here */
    session->gpsdata.separation      = getles32(bu, 38) / 100.0;
    /*@+type@*/
    session->newdata.altitude       -= session->gpsdata.separation;

    session->gpsdata.satellites_used = (int)bu[33];

    /*@-type@*//* splint has a bug here */
    session->gpsdata.dop.hdop        = getleu16(bu, 34) * 0.01;
    session->gpsdata.dop.pdop        = getleu16(bu, 36) * 0.01;
    /*@+type@*/
    mask                            |= DOP_SET;

    (void)strlcpy(session->gpsdata.tag, "129029", sizeof(session->gpsdata.tag));

    return mask | get_mode(session);
}


static gps_mask_t hnd_129038(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129039(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129040(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129794(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129798(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129802(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129809(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}


static gps_mask_t hnd_129810(unsigned char *bu, int len, PGN *pgn, struct gps_device_t *session)
{
    print_data(bu, len, pgn);
    gpsd_report(LOG_DATA, "pgn %6d(%3d):\n", pgn->pgn, session->driver.nmea2000.unit);
    return(0);
}

/*@-usereleased@*/
static const char msg_059392[] = {"ISO  Acknowledgment"};
static const char msg_060928[] = {"ISO  Address Claim"};
static const char msg_126208[] = {"NMEA Command/Request/Acknowledge"};
static const char msg_126464[] = {"ISO  Transmit/Receive PGN List"};
static const char msg_126992[] = {"GNSS System Time"};
static const char msg_126996[] = {"ISO  Product Information"};
static const char msg_129025[] = {"GNSS Position Rapid Update"};
static const char msg_129026[] = {"GNSS COG and SOG Rapid Update"};
static const char msg_129029[] = {"GNSS Positition Data"};
static const char msg_129539[] = {"GNSS DOPs"};
static const char msg_129540[] = {"GNSS Satellites in View"};
static const char msg_129038[] = {"AIS  Class A Position Report"};
static const char msg_129039[] = {"AIS  Class B Position Report"};
static const char msg_129040[] = {"AIS  Class B Extended Position Report"};
static const char msg_129794[] = {"AIS  Class A Static and Voyage Related Data"};
static const char msg_129798[] = {"AIS  SAR Aircraft Position Report"};
static const char msg_129802[] = {"AIS  Safty Related Broadcast Message"};
static const char msg_129809[] = {"AIS  Class B CS Static Data Report, Part A"};
static const char msg_129810[] = {"AIS  Class B CS Static Data Report, Part B"};
static const char msg_error [] = {"**error**"};

static PGN gpspgn[] = {{ 59392, 0, 0, hnd_059392, &msg_059392[0]},
		       { 60928, 0, 0, hnd_060928, &msg_060928[0]},
		       {126208, 0, 0, hnd_126208, &msg_126208[0]},
		       {126464, 1, 0, hnd_126464, &msg_126464[0]},
		       {126992, 0, 0, hnd_126992, &msg_126992[0]},
		       {126996, 1, 0, hnd_126996, &msg_126996[0]},
		       {129025, 0, 1, hnd_129025, &msg_129025[0]},
		       {129026, 0, 1, hnd_129026, &msg_129026[0]},
		       {129029, 1, 1, hnd_129029, &msg_129029[0]},
		       {129539, 0, 1, hnd_129539, &msg_129539[0]},
		       {129540, 1, 1, hnd_129540, &msg_129540[0]},
		       {0     , 0, 0, NULL,       &msg_error [0]}};

static PGN aispgn[] = {{ 59392, 0, 0, hnd_059392, &msg_059392[0]},
		       { 60928, 0, 0, hnd_060928, &msg_060928[0]},
		       {126208, 0, 0, hnd_126208, &msg_126208[0]},
		       {126464, 1, 0, hnd_126464, &msg_126464[0]},
		       {126992, 0, 0, hnd_126992, &msg_126992[0]},
		       {126996, 1, 0, hnd_126996, &msg_126996[0]},
		       {129038, 1, 2, hnd_129038, &msg_129038[0]},
		       {129039, 1, 2, hnd_129039, &msg_129039[0]},
		       {129040, 1, 2, hnd_129040, &msg_129040[0]},
		       {129794, 1, 2, hnd_129794, &msg_129794[0]},
		       {129798, 1, 2, hnd_129798, &msg_129798[0]},
		       {129802, 1, 2, hnd_129802, &msg_129802[0]},
		       {129809, 1, 2, hnd_129809, &msg_129809[0]},
		       {129810, 1, 2, hnd_129810, &msg_129810[0]},
		       {0     , 0, 0, NULL,       &msg_error [0]}};
/*@+usereleased@*/

/*@-immediatetrans@*/
static /*@null@*/ PGN *search_pgnlist(unsigned int pgn, PGN *pgnlist)
{
    int l1;
    PGN *work;

    l1 = 0;
    work = NULL;
    while (pgnlist[l1].pgn != 0) {
        if (pgnlist[l1].pgn == pgn) {
	    work = &pgnlist[l1];
	    break;
	} else {
	    l1 = l1 + 1;
	    }
	}
    return work;
}
/*@+immediatetrans@*/

static void vyspi_report_packet(struct gps_packet_t *pkg) {

  unsigned int ret = 0;

  while(ret < pkg->inbuflen)  {
    int i = 0;
    for (i = 0; i < 8; i++) {
      if(ret + i < pkg->inbuflen)
	printf("0x%02X ", (uint8_t)pkg->inbuffer[ret + i]);
      else
	printf("0x00 ");
    }
    printf(" : ");
    for (i = 0; i < 8; i++) {
      if(ret + i < pkg->inbuflen)
	printf("%c", 
	       pkg->inbuffer[ret + i] < 127 
	       && pkg->inbuffer[ret + i] > 31 ? pkg->inbuffer[ret + i] : '.');
      else
	printf(".");
    }
    ret += 8;
    printf("\n");
  }
}

static ssize_t vyspi_get(struct gps_device_t *session)
{   
  int fd = session->gpsdata.gps_fd;
  struct gps_packet_t * pkg = &session->packet;

  ssize_t          status;
  uint8_t len;

  gpsd_report(LOG_IO, "NMEA2000: reading data now @ inbuflen = %u\n",
	      pkg->inbuflen);

  errno = 0;
  status = read(fd, pkg->inbuffer + pkg->inbuflen,
		sizeof(pkg->inbuffer) - (pkg->inbuflen));
  pkg->outbuflen = 0;

  if(status == -1) {
    if ((errno == EAGAIN) || (errno == EINTR)) {
      gpsd_report(LOG_RAW + 2, "no bytes ready\n");
      len = 0;
      /* fall through, input buffer may be nonempty */
    } else {
      gpsd_report(LOG_RAW + 2, "errno: %s\n", strerror(errno));
      return -1;
    }
  } else {
    len = status;
    pkg->inbuflen += len;
  }

  if(len <= 0) {
    return 0;
  }

  //  vyspi_report_packet(pkg);

  /* Consume packet from the input buffer 
     - no lex parsing here to detect packet borders. 
     The SPI driver only sends one whole 
     packet at a time anyways. 

     This means that we'll loop on get/read until no more 
     data arrives and hand over to parse input */

  memcpy(session->packet.outbuffer, pkg->inbuffer, len);
  packet_reset(pkg);

  session->packet.outbuflen = len;
  session->packet.type = VYSPI_PACKET;

  gpsd_report(LOG_DATA, "VYSPI: len = %d, bytes= %u, errno= %d\n", 
	      len, status, errno);

  // printf("NMEA2000 get: exit(EXIT_SUCCESS)\n");
  return len;
}

// package types
#define PKG_TYPE_NMEA0183 0x01
#define PKG_TYPE_NMEA2000 0x02

/*@-mustfreeonly@*/
static gps_mask_t vyspi_parse_input(struct gps_device_t *session)
{    
  gps_mask_t mask = 0;
  struct gps_packet_t * pkg = &session->packet;

  PGN *work = NULL;

  static char * typeNames [] = {
    "UNKOWN", "NMEA0183", "NMEA2000"
  };

  uint8_t packet_len = pkg->outbuflen;
  uint8_t * buf = pkg->outbuffer;

  uint8_t len = 0;

  gpsd_report(LOG_INF, "VYSPI: parse_input called with packet len = %d\n",
	      packet_len);

  // one extra for reading both, len and type/origin
  while(len + 1 < packet_len) {

    uint8_t pkgType = (uint8_t)buf[len] & 0x0F;
    uint8_t pkgOrg  = (uint8_t)((buf[len] & 0xF0) >> 5);
    uint8_t pkgLen =  (uint8_t)buf[len + 1] & 0xFF;

    gpsd_report(LOG_DATA, "VYSPI: ptype= %s, org= %d, len= %d\n", 
		((pkgType > PKG_TYPE_NMEA2000) && (pkgType < PKG_TYPE_NMEA0183)) 
		? typeNames[0] : typeNames[pkgType], 
		pkgOrg, pkgLen);

    // skip 2 byte header now
    len += 2;
 
    if((pkgLen <= 0) || (packet_len <= len)) break;

    if(pkgType == PKG_TYPE_NMEA2000) {

      if(len + pkgLen + 8 >= packet_len) 
	break;

      uint32_t pgn = 
	  ((uint32_t)buf[len]) << 24
	| ((uint32_t)buf[len + 1]) << 16
	| ((uint32_t)buf[len + 2]) << 8
	| ((uint32_t)buf[len + 3]);

      uint32_t pkgid = 
	  ((uint32_t)buf[len + 4]) << 24
	| ((uint32_t)buf[len + 5]) << 16
	| ((uint32_t)buf[len + 6]) << 8
	| ((uint32_t)buf[len + 7]);

      gpsd_report(LOG_DATA, "VYSPI: PGN = %u, pid= %u, org= %u, len= %u\n", 
		  pgn, pkgid, pkgOrg, pkgLen);

      if (session->driver.nmea2000.pgnlist != NULL) {

	work = search_pgnlist(pgn, session->driver.nmea2000.pgnlist);

      } else {

	PGN *pgnlist;

	pgnlist = &gpspgn[0];
	work = search_pgnlist(pgn, pgnlist);
	if (work == NULL) {
	  pgnlist = &aispgn[0];
	  work = search_pgnlist(pgn, pgnlist);
	}
	if ((work != 0) && (work->type > 0)) {
	  session->driver.nmea2000.pgnlist = pgnlist;
	}
      }

      session->driver.nmea2000.workpgn = (void *) work;

      len += 8;

      if (work != NULL) {
	mask |= (work->func)(&session->packet.outbuffer[len], (int)pkgLen, work, session);
      }

      // length is packet length + 8 bytes pgn/pid
      len += pkgLen;

    } else if (pkgType == PKG_TYPE_NMEA0183) {
      
      if(len + pkgLen >= packet_len) 
	break;

      gps_mask_t st = 0;

      gpsd_report(LOG_DATA, "VYSPI: org= %d, len= %d\n", 
		  pkgOrg, pkgLen);

      char sentence[NMEA_MAX + 1];
      memset(sentence, 0, sizeof(sentence));
      memcpy(sentence, (char *)&session->packet.outbuffer[len], pkgLen);

      if (sentence[strlen(sentence)-1] != '\n')
	gpsd_report(LOG_IO, "<= GPS: %s\n", sentence);
      else
	gpsd_report(LOG_IO, "<= GPS: %s", sentence);

      if ((st= nmea_parse(sentence, session)) == 0) {
	gpsd_report(LOG_WARN, "unknown sentence: \"%s\"\n",	sentence);
      } 

      mask |= st;
	
      // length is packet length
      len += pkgLen;

    } else {

      gpsd_report(LOG_ERROR, "UNKOWN: len= %d\n", 
		  pkgLen);	  

      break;
    }
  }

    session->packet.outbuflen = 0;

    return mask;
}
/*@+mustfreeonly@*/

/*@+nullassign@*/

#ifndef S_SPLINT_S
int vyspi_open(struct gps_device_t *session) {

  char path[strlen(session->gpsdata.dev.path)], *port;
  socket_t dsock;
  (void)strlcpy(path, session->gpsdata.dev.path + 8, sizeof(path));
	
  session->gpsdata.gps_fd = -1;
  port = strchr(path, ':');

  if (port == NULL) {

    if(path[0] == '/')
      gpsd_report(LOG_INF, "Assuming SPI device.\n");

    dsock = open(path, O_RDWR | O_CREAT | O_TRUNC, 0);

  } else {

    *port++ = '\0';
    gpsd_report(LOG_INF, "opening TCP VYSPI feed at %s, port %s.\n", path,
		port);
    if ((dsock = netlib_connectsock(AF_UNSPEC, path, port, "tcp")) < 0) {
      gpsd_report(LOG_ERROR, "TCP device open error %s.\n",
		  netlib_errstr(dsock));
      return -1;
    } else
      gpsd_report(LOG_SPIN, "TCP device opened on fd %d\n", dsock);

  }

  gpsd_switch_driver(session, "VYSPI");
  session->gpsdata.gps_fd = dsock;
  session->sourcetype = source_can;
  session->servicetype = service_sensor;

  return session->gpsdata.gps_fd;
}

#endif /* of ifndef S_SPLINT_S */

/* *INDENT-OFF* */
const struct gps_type_t vyspi = {
    .type_name      = "VYSPI",       /* full name of type */
    .packet_type    = VYSPI_PACKET,	/* associated lexer packet type */
    .flags	    = DRIVER_NOFLAGS,	/* no rollover or other flags */
    .trigger	    = NULL,		/* detect their main sentence */
    .channels       = 12,		/* not an actual GPS at all */
    .probe_detect   = NULL,
    .get_packet     = vyspi_get,	/* how to get a packet */
    .parse_packet   = vyspi_parse_input,	/* how to interpret a packet */
    .rtcm_writer    = NULL,		/* Don't send RTCM to this */
    .event_hook     = NULL,
#ifdef RECONFIGURE_ENABLE
    .speed_switcher = NULL,		/* no speed switcher */
    .mode_switcher  = NULL,		/* no mode switcher */
    .rate_switcher  = NULL,		/* no rate switcher */
    .min_cycle      = 1,		/* nominal 1-per-second GPS cycle */
#endif /* RECONFIGURE_ENABLE */
#ifdef CONTROLSEND_ENABLE
    .control_send   = NULL,		/* how to send control strings */
#endif /* CONTROLSEND_ENABLE */
#ifdef NTPSHM_ENABLE
    .ntp_offset     = NULL,
#endif /* NTPSHM_ ENABLE */
};
/* *INDENT-ON* */

/* end */

#endif /* of  defined(NMEA2000_ENABLE) */
