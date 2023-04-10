#ifndef _SERIAL_CDC_H_
#define _SERIAL_CDC_H_

#include "stdio_t.h"

enum _RESPONSE_EVENT
{
	NORMAL_MSG_FROM_KLIPPER = 0,
	SHUTDOWN_EVENT_FROM_KLIPPER,
	SHUTDOWN_EVENT_FROM_ME,


};

enum _SERIAL_PTR
{
	SERIAL1_PTR = 0,
	SERIAL2_PTR ,
	ERROR_PTR ,
};
#define DEBUG_PORT SERIAL1_PTR
// #ifdef CONFIG_SPORT_B_DEBUG_ON_PORT0
// #define DEBUG_PORT SERIAL1_PTR
// #elif defined CONFIG_SPORT_B_DEBUG_ON_PORT1
// #define DEBUG_PORT SERIAL2_PTR
// #else
// #define DEBUG_PORT SERIAL1_PTR
// #endif
//_nor_serial_op *register_gcode_serial(void (*send_f)(unsigned char *,unsigned short));
_nor_serial_op *register_gcode_serial(unsigned char ptr,void (*send_f)(unsigned char *,unsigned short));
void send_gcode_message(unsigned char ptr,unsigned char  *buff , unsigned short  len);
void klipper_handle_response(uint8_t flag);

#endif
