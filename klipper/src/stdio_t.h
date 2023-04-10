#ifndef  _STDIO_T_H_
#define _STDIO_T_H_

//#include "generic/def.h"

typedef struct IO_OP
{
	void (*send_len_dat)(unsigned char *buff,unsigned short len);
	void (*recv_len_dat)(unsigned char *buff,unsigned short len);
	void (*notify)(unsigned char event);

}_nor_serial_op;

_nor_serial_op *register_io_serial(unsigned char ptr,void (*send_fuc)(unsigned char *,unsigned short));
void mprintf(unsigned char ptr,char *fmt,...);


#endif
