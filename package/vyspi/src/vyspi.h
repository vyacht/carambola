#ifndef _VYSPI_H_
#define _VYSPI_H_

#define VYSPI_READ   0x01
#define VYSPI_WRITE  0x02
#define VYSPI_RESET  0x04

#define VYSPI_HEADER 0x10
#define VYSPI_DATA   0x20
#define VYSPI_STATS  0x40
#define VYSPI_PARAM  0x80

#define VYSPI_PARAM_CHANGE_SPEED 0x01 // change speed of a (uart) device

#define VYSPI_IOC_CMD_DATA_SIZE 10

struct vyspi_ioc_cmd_t {

  uint8_t      type;   // change speed, etc.
  uint8_t      data[VYSPI_IOC_CMD_DATA_SIZE]; // data

};

#endif //
