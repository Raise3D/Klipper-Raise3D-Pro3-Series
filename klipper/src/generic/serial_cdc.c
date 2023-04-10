#include "stdio_t.h"
#include <string.h> // memmove
#include <stdlib.h>
#include "autoconf.h" // CONFIG_USB_VENDOR_ID
#include "atsam/gpio.h"
#include "atsam/internal.h" // gpio_peripheral
#include "board/pgm.h" // PROGMEMmak
#include "board/misc.h"
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "byteorder.h" // cpu_to_le16
#include "command.h" // output
#include "generic/usbstd.h" // struct usb_device_descriptor
#include "generic/usbstd_cdc.h" // struct usb_cdc_header_descriptor
#include "sched.h" // sched_wake_task
#include "generic/def.h"
#include "generic/serial_cdc.h"
#include "generic/msg_queue.h"
#include "generic/msg_event_t.h"
#include "stdio_t.h"
#include "usb_cdc.h"
#include "generic/refine_timer.h"

#define RMF500_TESTMODE
#define SUPPORT_LEN_TRANSFER 
//#define SUPPORT_RMF500
#define SUPPORT_PRO3
//#define SUPPORT_RMF500_M18

#if (CONFIG_ATSAM_PRO3_DOUBLE_GHEAD_PORT_0 == 1)
/* pro3  */
#define port_ptr  0
#elif (CONFIG_ATSAM_PRO3_DOUBLE_GHEAD_PORT_1 == 1)
#define port_ptr  1
#elif (CONFIG_ATSAM_DOUBLE_GHEAD == 1)
/* RMF500 double ghead */

#endif

static uint8_t version[]="0.1.0915";

extern void serial_init_t(uint8_t ptr,uint32_t baud);
static uint32_t cur_baud = 115200;
static uint8_t wcount[2] = { 0 };

enum _GHEAD_RESET_MODE
{
	ISP_MODE = 0,
	APP_MODE,
};
enum _CMD_LIST
{
	NORMAL_TEMP_CMD = 0,
	SET_FANSPEED_CMD ,
	CHECK_HEAD_S_CMD ,
	CHECK_SENSOR_S_CMD ,
	SET_24V_SWITCH_CMD ,
	CHECK_24V_SWITCH_CMD,
	SET_3V3_SWITCH_CMD ,
	CHECK_3V3_SWITCH_CMD,
	SET_AUTO_TEMP_CMD ,
	GET_VERSION_CMD,
	BTEST_CMD,

	PRO3_NORMAL_CMD ,
	PRO3_HEATEND_RD_CMD ,
	PRO3_HEATEND_RW_CMD ,
	PRO3_ERR_MODE_SET_CMD ,
	PRO3_FORMAT_E2PROM_CMD,
	PRO3_GHEAD_RESTART_EVENT,
	PRO3_GHEAD_RESTART_NOR_EVENT,
	CMD_END,
	ERROR_CMD ,
};

#define ERROR_VAL  -1
static void gcode_recv_dat(uint8_t *buff , uint16_t len);
static void gcode2_recv_dat(uint8_t *buff , uint16_t len);
static struct timer ghead_monitor_timer[2] = {0};
static struct timer ghead_reset_timer = {0};
#ifdef SUPPORT_RMF500
static struct timer xmotor_delay_timer = {0};
static struct timer ymotor_delay_timer = {0};
#endif



static _nor_serial_op gcode_serial_op[2] =
{
	{
		.send_len_dat    =    NULL,
		.recv_len_dat     =   gcode_recv_dat ,
		.notify                   =   NULL,
	},
	{
		.send_len_dat    =    NULL,
		.recv_len_dat     =   gcode2_recv_dat ,
		.notify                   =   NULL,
	},
};
typedef struct _FLOAT_DAT
{
	/* data */
	int upper ;
	int dow   ;
	int is_f  ;
	int is_m  ;
}_float_dat;

#define GHEAD_TIMEOUT_COUNT 5 
typedef struct GHEAD_P
{
	uint8_t ksesor_s;
	uint8_t blocation_s;
	int    temp_i;
	_float_dat temp_f;
	uint8_t is_ghead_in;
	uint8_t fan_speed ;
	int wait_count;
}_ghead_p;
_ghead_p  ghead_p[2] = { 0 };

#define MAX_RECV_LEN 128
static struct task_wake serial_task_wake;

_msg_queue *msg_box = NULL;

//static uint8_t serial_recbuff[MAX_RECV_LEN],rec_pos;

#define MAX_CMD_LEN 14
#define MAX_VAL_LEN 65
#define MAX_CMD_N   5
#define MAX_CRC_SIZE 8

typedef struct _CMD_VAL
{
        union
        {
                uint8_t cmd_val[MAX_VAL_LEN + 1]        ;
                struct
                {
                        uint8_t val_type ;
                        uint8_t val[MAX_VAL_LEN]         ;
                };
        };
        uint8_t val_ptr  ;
}_cmd_val;

typedef struct _CMD_IFO
{
        uint8_t sub_cmd;
        int sub_val ;
        uint8_t val[MAX_VAL_LEN] ;
		float sub_val_f;
		_float_dat float_dat;
}_cmd_ifo;
typedef struct _CMD_ANS
{
        uint8_t main_cmd ;
        _cmd_ifo cmd_ifo[MAX_CMD_N] ;
}_cmd_ans_ifo;
typedef struct _REC_CMD
{
        uint8_t cmd[16];
        uint8_t val[16];
        struct
        {
                uint8_t cmd_ptr : 4 ;
                uint8_t val_ptr : 4 ;
        };
//  uint8_t cmd_mask_f ;  
        uint8_t cmd_val_ptr;
        _cmd_val cmd_val[MAX_CMD_N];
		uint8_t cmd_rec_step  ;
		uint8_t cmd_rec_complete_flag ;
		uint8_t cmd_handle_busy ;

		/* only for crc */
		uint8_t cmd_rec_ptr ;
		uint8_t cmd_rec_buff[64];
		uint8_t crc_buff[MAX_CRC_SIZE];
		uint8_t is_crc  ;
		uint8_t crc_ptr ;
}_rec_cmd;

static _rec_cmd rec_cmd[2] = {0};
static _cmd_ans_ifo cmd_ans_ifo[2] = {0};

static void cmd_rec(uint8_t ptr,uint8_t dat);
static void cmd_handle(uint8_t ptr);
static void complete_rec(uint8_t ptr);
static void cmd_reset_rec(uint8_t ptr);
static uint8_t cmd_ans_new(uint8_t ptr,_cmd_ans_ifo *ansifo);
static void recv_dat(uint8_t serial_ptr,uint8_t dat);
static void refresh_ghead_in_status(uint8_t ptr);
static void report_ghead_in_s(uint8_t ptr);
static void wake_serialtask(void);

void close_heat_bed(void);

#ifdef SUPPORT_LEN_TRANSFER
static void send_len_data_to_host(uint8_t ptr,uint8_t mode,uint8_t addr,uint8_t len,uint8_t *val);
#endif

#define MONITOR_PER_COUNT 1000
timer_handler monitor_timer[2] = { NULL } ;

uint32_t led_pin = GPIO('B',14);
struct gpio_out g_led;
#ifdef SUPPORT_PRO3
uint32_t heater_bed_pin = GPIO('A',19);
uint32_t l_led_pin = GPIO('D',18);
uint32_t r_led_pin = GPIO('C',28);
uint32_t ghead_reset_pin = GPIO('C',2);
uint32_t ghead_boot_pin = GPIO('E',3);

struct gpio_out g_led;
struct gpio_out r_led_light;
struct gpio_out l_led_light;
struct gpio_out bed_heater;

struct gpio_out ghead_boot;
struct gpio_out ghead_reset;
#endif



#ifdef SUPPORT_RMF500
uint32_t heater_bed_pin = GPIO('D',18);
uint32_t probe_pin = GPIO('C',2);
uint32_t motor_en_pin = GPIO('E',2);
uint32_t motor1_en_pin = GPIO('E',1);
uint32_t z_lock_pin = GPIO('A',8);

uint32_t x_motor_aaf_pin = GPIO('C',4);
uint32_t y_motor_aaf_pin = GPIO('A',17);

uint32_t x_motor_clearaff_pin = GPIO('A',15);
uint32_t y_motor_clearaff_pin = GPIO('D',20);

uint32_t x_motor_status_pin = GPIO('E',4);
uint32_t y_motor_status_pin = GPIO('D',30);

struct gpio_in x_motor_aaf ;
struct gpio_in y_motor_aaf ;

struct gpio_in x_motor_status ;
struct gpio_in y_motor_status ;

struct gpio_out x_motor_clearaff;
struct gpio_out y_motor_clearaff;

struct gpio_out probe;
struct gpio_out motor;
struct gpio_out motor1;
struct gpio_out z_lock;
struct gpio_out bed_heater;
#endif

static void monitor_ch_fuc(uint8_t ptr)
{
	gpio_out_toggle_noirq(g_led);
	if(ptr > SERIAL2_PTR)
		return ;
	if(ghead_p[ptr].is_ghead_in == true)
	{
		wcount[ptr] = 0;
		//mprintf(DEBUG_PORT,"monitor chn -- %d count -- %d \r\n",ptr,ghead_p[ptr].wait_count);
		ghead_p[ptr].wait_count++;
		if(ghead_p[ptr].wait_count >= GHEAD_TIMEOUT_COUNT)
		{
			ghead_p[ptr].wait_count = 0;
			if(ghead_p[ptr].is_ghead_in == true)
			{
				//mprintf(DEBUG_PORT,"ghead is not in\r\n");
				ghead_p[ptr].is_ghead_in = false;
				/* awake task to send .. */
				report_ghead_in_s(ptr);
			}	
		}
	}
	else
	{
		wcount[ptr]++;
		if(wcount[ptr] > 3)
		{
			wcount[ptr] = 0;
			switch(cur_baud)
			{
				case 19200:
					cur_baud = 115200;
					break;
				case 115200:
					cur_baud = 230400;
					break;
				case 230400:
					cur_baud = 19200;
					break;
				default:
					cur_baud = 19200;
					break;
			}
			serial_init_t(ptr,cur_baud);
		}
	}
}
void klipper_handle_response(uint8_t flag)
{
	switch(flag)
	{
		case NORMAL_MSG_FROM_KLIPPER:
		/* normal msg from upper layer */
			break;
		case SHUTDOWN_EVENT_FROM_KLIPPER:
		/* shutdown event from upper layer */
			close_heat_bed();
#ifdef SUPPORT_RMF500
			gpio_out_write(z_lock,0);
#endif
			break;
		case SHUTDOWN_EVENT_FROM_ME:
		/* shutdown event from myself */
			close_heat_bed();
#ifdef SUPPORT_RMF500
			gpio_out_write(z_lock,0);
#endif
			break;
		default:
			break;
	}
}
#ifdef SUPPORT_RMF500
static uint_fast8_t clear_x_aaf(struct timer* tim)
{
	gpio_out_write(x_motor_clearaff,1);
	return SF_DONE;
}
void start_delay_xaaf_clear(uint32_t count)
{
	xmotor_delay_timer.func = clear_x_aaf;
	xmotor_delay_timer.waketime = timer_from_us(count);
	sched_add_timer(&xmotor_delay_timer);
}
static uint_fast8_t clear_y_aaf(struct timer* tim)
{
	gpio_out_write(y_motor_clearaff,1);
	return SF_DONE;
}
void start_delay_yaaf_clear(uint32_t count)
{
	ymotor_delay_timer.func = clear_y_aaf;
	ymotor_delay_timer.waketime = timer_from_us(count);
	sched_add_timer(&ymotor_delay_timer);
}
#endif
static uint_fast8_t monitor_ghead1(struct timer* tim)
{
	send_msg(msg_box,SERIAL_MONITOR_CHN1_EVENT);
	ghead_monitor_timer[SERIAL1_PTR].waketime += timer_from_us(1000000);
	return SF_RESCHEDULE;
}
static uint_fast8_t monitor_ghead2(struct timer* tim)
{
	send_msg(msg_box,SERIAL_MONITOR_CHN2_EVENT);
	ghead_monitor_timer[SERIAL2_PTR].waketime += timer_from_us(1000000);
	return SF_RESCHEDULE;
}
void start_monitor_ghead(uint8_t ptr,uint32_t is_now)
{
	ghead_monitor_timer[ptr].func = ptr == SERIAL1_PTR ? monitor_ghead1 : monitor_ghead2;
	if(is_now == 0)
		ghead_monitor_timer[ptr].waketime = timer_from_us(1000000);
	else
		ghead_monitor_timer[ptr].waketime = timer_from_us(is_now);
	sched_add_timer(&ghead_monitor_timer[ptr]);
}
#ifdef SUPPORT_PRO3 
static uint_fast8_t  reset_ghead(struct timer* tim)
{
	gpio_out_write(ghead_reset,0);
	return SF_DONE;
}
void start_reset_ghead(uint8_t mode)
{
	if(mode == ISP_MODE)
	{
		gpio_out_write(ghead_boot,1);
	}
	else
	{
		gpio_out_write(ghead_boot,0);
	}
	gpio_out_write(ghead_reset,1);
	ghead_reset_timer.func = reset_ghead;
	ghead_reset_timer.waketime = timer_from_us(300000);
	sched_add_timer(&ghead_reset_timer);
}
#endif
void close_heat_bed(void)
{
#ifdef SUPPORT_RMF500	
	gpio_out_write(bed_heater,1);
#else
	gpio_out_write(bed_heater,0);
#endif
}
void init_serial_cdc(void)
{	/* init flash led port */
	//gpio_peripheral(led_pin, 'A', 1);
	g_led = gpio_out_setup(led_pin,1);
	bed_heater = gpio_out_setup(heater_bed_pin,0);
	close_heat_bed();
#ifdef SUPPORT_PRO3 
	r_led_light = gpio_out_setup(r_led_pin,1);
	l_led_light = gpio_out_setup(l_led_pin,1);
	ghead_boot = gpio_out_setup(ghead_boot_pin,0);
	ghead_reset = gpio_out_setup(ghead_reset_pin,1);
	start_reset_ghead(APP_MODE);
#endif
#ifdef SUPPORT_RMF500	 
	// //gpio_in_setup
	// x_motor_aaf = gpio_in_setup(x_motor_aaf_pin,1);
	// y_motor_aaf = gpio_in_setup(y_motor_aaf_pin,1);

	// x_motor_status = gpio_in_setup(x_motor_status_pin,1);
	// y_motor_status = gpio_in_setup(y_motor_status_pin,1);

	x_motor_clearaff = gpio_out_setup(x_motor_status_pin,1);
	y_motor_clearaff = gpio_out_setup(y_motor_status_pin,1);
	gpio_out_write(x_motor_clearaff,1);
	gpio_out_write(y_motor_clearaff,1);

	motor = gpio_out_setup(motor_en_pin,0);
	motor1 = gpio_out_setup(motor1_en_pin,0);
	z_lock = gpio_out_setup(z_lock_pin,0);
	probe = gpio_out_setup(probe_pin,0);
	gpio_out_write(probe,0);
	gpio_out_write(z_lock,0);
	gpio_out_write(motor,0);
	gpio_out_write(motor1,0);
#endif
//	 init_msg_list();
	start_monitor_ghead(port_ptr,0);
	msg_box = create_msg_box(wake_serialtask);
}
DECL_INIT(init_serial_cdc);
_nor_serial_op *register_gcode_serial(unsigned char ptr,void (*send_f)(unsigned char *,unsigned short))
{
	gcode_serial_op[ptr].send_len_dat = send_f;
	return &gcode_serial_op[ptr];
}
static void wake_serialtask(void)
{
	sched_wake_task(&serial_task_wake);
}
static void gcode_recv_dat(uint8_t *buff , uint16_t len)
{
	for(uint8_t i = 0; i < len ; i ++)
	{
		recv_dat(SERIAL1_PTR,buff[i]);
	}
}
 static void gcode2_recv_dat(uint8_t *buff , uint16_t len)
 {
	for(uint8_t i = 0; i < len ; i ++)
	{
		recv_dat(SERIAL2_PTR,buff[i]);	
	}
 }

 void send_gcode_message(uint8_t ptr ,uint8_t *buff , uint16_t len)
{
	if(gcode_serial_op[ptr].send_len_dat)
	{
		gcode_serial_op[ptr].send_len_dat(buff,len);
	}
}

static void recv_dat(uint8_t serial_ptr,uint8_t dat)
{
	/**/
	cmd_rec(serial_ptr,dat);
}
char getc[] = "get ok"; 
void serial_cdc_run_task(void)
{
	if (!sched_check_wake(&serial_task_wake))
          return;
	int msg = read_msg(msg_box);
	switch (msg)
	{
	case SERIAL_RECV_CHN1_EVENT:
		cmd_handle(SERIAL1_PTR);
		break;
	case SERIAL_RECV_CHN2_EVENT:
		cmd_handle(SERIAL2_PTR);
		break;
	case SERIAL_MONITOR_CHN1_EVENT:
		monitor_ch_fuc(SERIAL1_PTR);
		break;
	case SERIAL_MONITOR_CHN2_EVENT:
		monitor_ch_fuc(SERIAL2_PTR);
		break;
	default:
		//mprintf(DEBUG_PORT,"error msg\r\n");
		break;
	}
}
static uint8_t crc_cal(uint8_t *dat ,uint8_t len)
{
	uint8_t crc = 0;
	for(uint8_t i = 0; i < len ; i++)
	{
		crc ^= dat[i];
	}
	return crc;
}
DECL_TASK(serial_cdc_run_task);
static _float_dat my_atof(const char *str,uint8_t n)
{
	uint8_t dot_ptr = 0;
	uint8_t dot_n = 0;
	int uper = 0;
	int dow = 0;
	uint8_t d_pos_f = false;
	_float_dat val = { 0 };
	for(uint8_t i = 0; i < n ; i ++ )
	{
		char c = str[i];
		if(c >= '0' && c <= '9')
		{
			if(c >= '1' && d_pos_f == false)
				d_pos_f = true;
			if(dot_n == 0)
			{
				uper = uper*10 + (c - '0'); 
			}
			else if(dot_n == 1)
			{
				//dow = dow + ((float)(c - '0'))/(float)spr_c(10,dot_ptr);
				dow = dow*10 + (c - '0');
				//uper = uper*10 + (c - '0'); 
				dot_ptr ++;
			}	
		}
		else if(c == '.')
		{
			dot_n++;
			dot_ptr = 0;
			if(dot_n >= 2)
			{
				goto t_end;
			}
		}
		else if (c == '-' && i == 0)
		{
			/* code */
			val.is_m = 1;
		}
		else if(c == ' '&& d_pos_f == false)
		{
			continue ;
		}
		else
		{
			goto t_end;
		}
	}
t_end:
	val.upper = uper;
	val.dow = dow;
	val.is_f = dot_ptr > 0 ? 1 : 0 ;
	return val;
}
static uint8_t cmd_ans_new(uint8_t ptr,_cmd_ans_ifo *ansifo)
{
	for(uint8_t i = 0; i < MAX_CMD_N ; i++)
	{
		if(rec_cmd[ptr].cmd_val[i].val_type != 0 && rec_cmd[ptr].cmd_val[i].val_ptr > 0)
		{
			ansifo->cmd_ifo[i].sub_cmd = rec_cmd[ptr].cmd_val[i].val_type;
			memcpy(ansifo->cmd_ifo[i].val,rec_cmd[ptr].cmd_val[i].val,strlen((const char *)rec_cmd[ptr].cmd_val[i].val));
			if(((rec_cmd[ptr].cmd_val[i].val[0] >= '0' && rec_cmd[ptr].cmd_val[i].val[0] <= '9')||rec_cmd[ptr].cmd_val[i].val[0]  == '-') && strlen((const char *)rec_cmd[ptr].cmd_val[i].val) < 10)
			{
				/* get a number value */ 
				ansifo->cmd_ifo[i].sub_val = (int)atoi((const char *)&rec_cmd[ptr].cmd_val[i].val);
				ansifo->cmd_ifo[i].float_dat = (_float_dat)my_atof((const char *)&rec_cmd[ptr].cmd_val[i].val,20);
			}
			else if(rec_cmd[ptr].cmd_val[i].val[0] >= 'A' && rec_cmd[ptr].cmd_val[i].val[0] <='Z' )
			{
				/* get other value maybe asc */
				ansifo->cmd_ifo[i].sub_val = rec_cmd[ptr].cmd_val[i].val[0];
			}
		}
	}
	if(strcmp((const char *)rec_cmd[ptr].cmd,"M5000") == 0)
	{
		ansifo->main_cmd = NORMAL_TEMP_CMD;
		return NORMAL_TEMP_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5001") == 0 )
	{
		ansifo->main_cmd   = SET_FANSPEED_CMD;
		return SET_FANSPEED_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5002") == 0 )
	{
		ansifo->main_cmd   = CHECK_HEAD_S_CMD;
		return CHECK_HEAD_S_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5003") == 0 )
	{
		ansifo->main_cmd   = CHECK_SENSOR_S_CMD;
		return CHECK_SENSOR_S_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5004") == 0 && rec_cmd[ptr].cmd_val[0].val_type == 0 && rec_cmd[ptr].cmd_val[0].val_ptr == 0)
	{
		ansifo->main_cmd   = CHECK_24V_SWITCH_CMD;
		return CHECK_24V_SWITCH_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5004") == 0 && rec_cmd[ptr].cmd_val[0].val_type == 'S' && rec_cmd[ptr].cmd_val[0].val_ptr > 0)
	{
		ansifo->main_cmd   = SET_24V_SWITCH_CMD;
		return SET_24V_SWITCH_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5005") == 0 && rec_cmd[ptr].cmd_val[0].val_type == 0 && rec_cmd[ptr].cmd_val[0].val_ptr == 0)
	{
		ansifo->main_cmd   = CHECK_3V3_SWITCH_CMD;
		return CHECK_3V3_SWITCH_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5005") == 0 && rec_cmd[ptr].cmd_val[0].val_type == 'S' && rec_cmd[ptr].cmd_val[0].val_ptr > 0)
	{
		ansifo->main_cmd   = SET_3V3_SWITCH_CMD;
		return SET_3V3_SWITCH_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M9999") == 0 )
	{
		ansifo->main_cmd   = SET_AUTO_TEMP_CMD;
		return SET_AUTO_TEMP_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5100") == 0 )
	{
		ansifo->main_cmd   = PRO3_NORMAL_CMD;
		return PRO3_NORMAL_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5105") == 0 )
	{
		ansifo->main_cmd   = PRO3_GHEAD_RESTART_EVENT;
		return PRO3_GHEAD_RESTART_EVENT;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5106") == 0 )
	{
		ansifo->main_cmd   = PRO3_HEATEND_RD_CMD;
		return PRO3_HEATEND_RD_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5107") == 0 )
	{
		ansifo->main_cmd   = PRO3_HEATEND_RW_CMD;
		return PRO3_HEATEND_RW_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5108") == 0 )
	{
		ansifo->main_cmd   = PRO3_FORMAT_E2PROM_CMD;
		return PRO3_FORMAT_E2PROM_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M5109") == 0 )
	{
		ansifo->main_cmd   = PRO3_ERR_MODE_SET_CMD;
		return PRO3_ERR_MODE_SET_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"M115") == 0 )
	{
		ansifo->main_cmd   = GET_VERSION_CMD;
		return GET_VERSION_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"MTEST") == 0 )
	{
		ansifo->main_cmd   = BTEST_CMD;
		return BTEST_CMD;
	}
	else if(strcmp((const char *)rec_cmd[ptr].cmd,"MSTART") == 0 )
	{
		ansifo->main_cmd   = PRO3_GHEAD_RESTART_NOR_EVENT;
		return PRO3_GHEAD_RESTART_NOR_EVENT;
	}
	return ERROR_CMD;
}
char test[] = "test..\r\n";
static int gcode_seen(uint8_t ptr,char c)
{
	for(uint8_t i = 0; i < MAX_CMD_N ; i++)
	{
		if(cmd_ans_ifo[ptr].cmd_ifo[i].sub_cmd == c)
		{
			return cmd_ans_ifo[ptr].cmd_ifo[i].sub_val;
		}
	}
	return  ERROR_VAL;
}
static int gcode_seen_f(uint8_t ptr,char c,_float_dat *f_dat)
{
	for(uint8_t i = 0; i < MAX_CMD_N ; i++)
	{
		if(cmd_ans_ifo[ptr].cmd_ifo[i].sub_cmd == c)
		{
			f_dat->upper = cmd_ans_ifo[ptr].cmd_ifo[i].float_dat.upper;
			f_dat->is_f  = cmd_ans_ifo[ptr].cmd_ifo[i].float_dat.is_f ;
			f_dat->dow  = cmd_ans_ifo[ptr].cmd_ifo[i].float_dat.dow ;
			f_dat->is_m = cmd_ans_ifo[ptr].cmd_ifo[i].float_dat.is_m ;
			return 0;
		}
	}
	return  ERROR_VAL;
}
static int get_x_bit(int dat , int pos)
{
	if(dat < 10)
	{
		if(pos == 0)
			return dat;
		return 0;
	}
	else if(dat >= 10 && dat < 100)
	{
		if(pos == 0)
			return dat/10;
		else if(pos == 1)
			return dat%10;
		else 
			return 0;
	}
	else if(dat >= 100 && dat < 1000)
	{
		if(pos == 0)
			return dat/100;
		else if(pos == 1)
			return dat%100/10;
		else if(pos == 1)
			return dat%100%10;
		else 
			return 0;
	}
	return 0;
}
static int m_ftoi(_float_dat f_dat,uint8_t pix_n)
{
	int val = f_dat.upper;
	switch (pix_n)
	{
	case 1:
		val = f_dat.upper;
		break;
	case 10:
		val = f_dat.upper*10;
		val += get_x_bit(f_dat.dow,0);
		break;
	case 100:
		val = f_dat.upper*100;
		val += (get_x_bit(f_dat.dow,0)*10);
		val += (get_x_bit(f_dat.dow,1)); 
		break;
	default:
		break;
	}
	if(f_dat.is_m == 0)
		return val;
	else 
		return (0 - val);
}

//static void report_ghead_para(uint8_t ptr)
//{
//	sendf("ghead_para gh_ptr=%c sensor=%c location=%c value=%hu",ptr,ghead_p[ptr].ksesor_s,ghead_p[ptr].blocation_s,ghead_p[ptr].temp_i);
//}
static void report_ghead_temp(uint8_t ptr)
{
	uint32_t clock = timer_read_time();
	sendf("ghead_temp gh_ptr=%c  value=%hu clock=%u",ptr,ghead_p[ptr].temp_i,clock);
}
static void report_ghead_location_s(uint8_t ptr)
{
	uint32_t clock = timer_read_time();
	sendf("ghead_location_s gh_ptr=%c location=%c clock=%u",ptr,ghead_p[ptr].blocation_s,clock);
}
static void report_ghead_ksensor_s(uint8_t ptr)
{
	uint32_t clock = timer_read_time();
	sendf("ghead_ksensor_s gh_ptr=%c sensor=%c clock=%u",ptr,ghead_p[ptr].ksesor_s,clock);
}
static void report_ghead_in_s(uint8_t ptr)
{
	uint32_t clock = timer_read_time();
	sendf("ghead_in_s gh_ptr=%c ghead_s=%c clock=%u",ptr,ghead_p[ptr].is_ghead_in,clock);
}
static void reponse_fanspeed_set_s(uint8_t ptr)
{
	uint32_t clock = timer_read_time();
	sendf("ghead_fanset_s gh_ptr=%c f_speed=%u clock=%u",ptr,ghead_p[ptr].fan_speed,clock);
}
static void repond_fuc(uint8_t ptr,uint8_t mode,uint8_t val)
{
	uint32_t clock = timer_read_time();
	sendf("ghead_respond_s gh_ptr=%c mode=%c val=%c clock=%u",ptr,mode,val,clock);
}
#ifdef SUPPORT_LEN_TRANSFER
static void send_len_data_to_host(uint8_t ptr,uint8_t mode,uint8_t addr,uint8_t len,uint8_t *val)
{
	//uint32_t clock = timer_read_time();
	sendf("ghead_slen gh_ptr=%c mode=%c addr=%c len=%c buff=%*s",ptr,mode,addr,len,len,val);
}
#endif
static void refresh_ghead_in_status(uint8_t ptr)
{
	ghead_p[ptr].wait_count = 0;
	if(ghead_p[ptr].is_ghead_in == false)
	{
		ghead_p[ptr].is_ghead_in = true;
		wcount[ptr] = 0;
		//mprintf(DEBUG_PORT,"ghead is comming in\r\n");
		report_ghead_in_s(ptr);
	}
}
void set_fan_speed(uint32_t *args)
{
	uint8_t ptr = args[0];
	uint8_t fan_speed = args[1];
	if(ptr <=1 && fan_speed <= 100)
	{
		//mprintf(port_ptr,"M5001 S%d\r\n",fan_speed);
	}
}
/* only for pro3 */
void set_heater_switch(uint32_t *args)
{
	uint8_t ptr = args[0];
	uint8_t sta = args[1];
	if(ptr <=1 && sta <= 1)
	{
		mprintf(port_ptr,"M5100 H%d H%d\r\n",ptr,sta);
	}
}
void set_probe(uint32_t *args)
{
	uint8_t val = args[0];
	if(val < 2)
		mprintf(port_ptr,"M5100 T0 D%d\r\n",val);
}
void set_normal_cmd(uint32_t *args)
{
	uint8_t mode = args[0];
	uint8_t ptr  = args[1];
	uint32_t val  = args[2];
	switch (mode)
	{
	case 'H':
		/* code switch heaters */
		mprintf(port_ptr,"M5100 H%d H%d\r\n",ptr,val);
		break;
	case 'S':
		if(val == 0)
		{
			/* left .. */
			mprintf(port_ptr,"M5100 S0 W1940\n");
		}
		else
		{
			mprintf(port_ptr,"M5100 S0 W1191\n");
		}
		break;
	case 'S' + 'T':
		mprintf(port_ptr,"M5100 H0 A0\n");
		mprintf(port_ptr,"M5100 H1 A0\n");
		break;
	case 'T':
#ifdef SUPPORT_RMF500
 		
#ifdef SUPPORT_RMF500_M18
 		mprintf(port_ptr,"M5100 T0 D%d\n",val);
#else
		gpio_out_write(probe,val);
#endif
 #else
		mprintf(SERIAL1_PTR,"M5100 T0 D%d\n",val);
		if(val == 0)
		{
			mprintf(port_ptr,"M5100 T0 D0\n",val);
		}
		else
		{
			mprintf(port_ptr,"M5100 T0 D1\n",val);
		}
 #endif	
		repond_fuc(ptr,mode,val);
		break;
	case 'B':
		switch(val)
		{
			case 0:
			/* 9600 */
				cur_baud = 9600;
				break;
			case 1:
			/* 19200 */
				cur_baud = 19200;
				break;
			case 2:
			/* 38400 */
				cur_baud = 38400;
				break;
			case 3:
			/* 57600 */
				cur_baud = 57600;
				break;
			case 4:
			/* 115200 */
				cur_baud = 115200;
				break;
			case 5:
			/* 230400 */
				cur_baud = 230400;
				break;
			default:
				break;
		}
		serial_init_t(port_ptr,cur_baud);
		repond_fuc(ptr,mode,val);
		break;
	case 'B' + 'R' + 'T':
		usb_request_bootloader();
		break;
	case 'D':
		break;
	case 'L':
#ifdef SUPPORT_PRO3
		if(val == 0)
		{
			gpio_out_write(r_led_light,0);
			gpio_out_write(l_led_light,0);
		}	
		else
		{
			gpio_out_write(r_led_light,1);
			gpio_out_write(l_led_light,1);
		}
#endif
		break;
	case 'A':
	{
		uint8_t addr = (val >> 8); 
		uint8_t size = (uint8_t)val;
		mprintf(port_ptr,"M5106 H%d A%x S%d\n",ptr,addr,size);
		// uint8_t addr = 21;
		// uint8_t mode = 'R' + 'H';
		// uint8_t test_str[] = "snidejiusnide";
		// send_len_data_to_host(ptr,mode,addr,strlen((const char *)test_str),test_str);
	}
		break;
	case 'R' + 'H':
		/* reset ghead */
		if(val < 2)
		{
			/* 0 isp mode 1 app mode */
			start_reset_ghead(val);
		}
		break;
	case 'C' + 'V':
		mprintf(port_ptr,"M5100 CV\n");
		break;
	case 'V':
		// move_board ver
		send_len_data_to_host(0,'V',0,strlen((const char *)version),version);
		break;
#ifdef SUPPORT_RMF500
	case 'Z':
		gpio_out_write(z_lock,val);
		repond_fuc(ptr,mode,val);
		break;
	case 'C':
		switch(ptr)
		{
			case 0:
				/* x head */
				gpio_out_write(x_motor_clearaff,val);
				if(val == 1)
				{
					start_delay_xaaf_clear(500);
				}
				break;
			case 1:
				gpio_out_write(y_motor_clearaff,val);
				if(val == 1)
				{
					start_delay_yaaf_clear(500);
				}
				break;
			default:
				break;
		}
		repond_fuc(ptr,mode,val);
		break;
	case 'U':
		switch(ptr)
		{
			case 0:
				/* x */
				repond_fuc(ptr,mode,gpio_in_read(x_motor_status));
				break;
			case 1:
				/* y */
				repond_fuc(ptr,mode,gpio_in_read(y_motor_status));
				break;
		}
		break;
#endif
	default:
		break;
	}
}
/**************************/
void query_ghead_in_s(uint32_t *args)
{
	uint8_t ptr = args[0];
	if(ptr <= 1)
	{
		report_ghead_in_s(ptr);
	}
}
void start_monitor_ghead_cmd(uint32_t *args)
{
	uint8_t ptr = args[0];
	if(ptr <= SERIAL2_PTR)
	{
		 start_monitor_ghead(ptr,500000);
	}
	else if(ptr == 2)
	{
		start_monitor_ghead(SERIAL1_PTR,500000);
		start_monitor_ghead(SERIAL2_PTR,700000);
	}	
}
#ifdef SUPPORT_LEN_TRANSFER
uint8_t gbuff[64] = {0};
void trans_len_data(uint32_t *args)
{
	uint8_t ptr = args[0];
	uint8_t mode = args[1];
	uint8_t addr = args[2];
	uint8_t len = args[3];
	memset(gbuff,0,64);
    uint8_t *data = command_decode_ptr(args[4]);
	uint8_t i = 0;
	for(i = 0;i < len ;i++)
	{
		gbuff[i] = data[i];	
	}
	switch(mode)
	{
		case 'H':
			mprintf(port_ptr,"M5107 H%d A%d H%s\n",ptr,addr,gbuff);
			mprintf(SERIAL1_PTR,"M5107 H%d A%d H%s\n",ptr,addr,gbuff);
			break;
		case 'D':
			mprintf(SERIAL1_PTR,"M5107 H%d A%d D%s\n",ptr,addr,gbuff);
			mprintf(port_ptr,"M5107 H%d A%d H%s\n",ptr,addr,gbuff);
			break;
		default:
			break;
	}
	// uint8_t mode_t = 'W' + mode;
	// send_len_data_to_host(ptr,mode_t,addr,strlen((const char *)gbuff),gbuff);
}
#endif
DECL_COMMAND_FLAGS(start_monitor_ghead_cmd,1,"ghead_start_monitor_ghead ptr=%c");
DECL_COMMAND_FLAGS(query_ghead_in_s,1,"ghead_query_in ptr=%c");
DECL_COMMAND_FLAGS(set_fan_speed,1,"ghead_set_fan_speed ptr=%c fans=%u");
DECL_COMMAND_FLAGS(set_heater_switch,1,"ghead_set_heater_s ptr=%c h_s=%u");
DECL_COMMAND_FLAGS(set_normal_cmd,1,"ghead_set_normal_cmd mode=%c ptr=%c h_s=%u");
#ifdef SUPPORT_LEN_TRANSFER 
DECL_COMMAND_FLAGS(trans_len_data,1,"ghead_set_len_dat ptr=%c mode=%c addr=%c buff=%*s");
#endif
static uint16_t char2hex(const char *str,const uint8_t n)
{
	uint16_t hex = 0;
	uint8_t i ;
	for(i = 0 ; i < n ; i++)
	{
		hex <<= (i == 0 ? 0 : 4) ;
		if(str[i] >= '0' && str[i] <='9')
		{
			uint16_t val = str[i] - '0';
			hex |= val;
		}
		else if(str[i] >= 'a' && str[i] <='f')
		{
			uint16_t val = str[i] - 'a' + 10;
			hex |= val;
		}
		else if(str[i] >= 'A' && str[i] <='F')
		{
			uint16_t val = str[i] - 'A' + 10;
			hex |= val;
		}
		else
		{
			/* wrong asc */
			return 0;
		}
	}
	return hex;
}
static void cmd_handle(uint8_t ptr)
{
	if(rec_cmd[ptr].cmd_rec_complete_flag == true)  
	{
		if(rec_cmd[ptr].is_crc == true)
		{
			/*  */
			uint8_t r_crc = (uint8_t)atoi((const char *)rec_cmd[ptr].crc_buff);
            uint8_t t_crc = crc_cal(rec_cmd[ptr].cmd_rec_buff,rec_cmd[ptr].cmd_rec_ptr);
			if(r_crc != t_crc)
			{
				goto h_end;
			}
		}
		memset(&cmd_ans_ifo[ptr],0,sizeof(_cmd_ans_ifo));
		if(cmd_ans_new(ptr,&cmd_ans_ifo[ptr]) != ERROR_CMD)
		{
			refresh_ghead_in_status(ptr);
			switch(cmd_ans_ifo[ptr].main_cmd)
			{
				case NORMAL_TEMP_CMD: 
					{
						int k_s = 0;
						int res = 0;
#ifdef RMF500_TESTMODE
						if((res = gcode_seen_f(ptr,'S',&ghead_p[SERIAL1_PTR].temp_f)) != ERROR_VAL)
						{
							ghead_p[SERIAL1_PTR].temp_i = m_ftoi(ghead_p[SERIAL1_PTR].temp_f,10);
							report_ghead_temp(SERIAL1_PTR);
						}
						if((k_s = gcode_seen(ptr,'K')) != ERROR_VAL)
						{
							ghead_p[ptr].ksesor_s = k_s == 0 ? true : false;
							report_ghead_ksensor_s(SERIAL1_PTR);
						}
#else
						if((res = gcode_seen_f(ptr,'S',&ghead_p[ptr].temp_f)) != ERROR_VAL)
						{
							ghead_p[ptr].temp_i = m_ftoi(ghead_p[ptr].temp_f,10);
							#if 0
							ghead_p[ptr].temp_i = ghead_p[ptr].temp_f.is_f == 0 ? ghead_p[ptr].temp_f.upper : (ghead_p[ptr].temp_f.upper*10 + ghead_p[ptr].temp_f.dow);
							mprintf(DEBUG_PORT,"temp = %d\r\n",ghead_p[ptr].temp_i);
							if(ghead_p[ptr].temp_f.is_f)
								mprintf(DEBUG_PORT,"temp f = %d.%d\r\n",ghead_p[ptr].temp_f.upper,ghead_p[ptr].temp_f.dow);
							else
								mprintf(DEBUG_PORT,"temp f = %d\r\n",ghead_p[ptr].temp_f.upper);
							#endif
							report_ghead_temp(ptr);
						}
						if((k_s = gcode_seen(ptr,'K')) != ERROR_VAL)
						{
							ghead_p[ptr].ksesor_s = k_s == 0 ? true : false;
							report_ghead_ksensor_s(ptr);
						}
#endif
					}
					break;
				case CHECK_HEAD_S_CMD:
					{
						int location_s = 0;
						if((location_s = gcode_seen(ptr,'S')) != ERROR_VAL)
						{
#ifdef RMF500_TESTMODE
							ghead_p[SERIAL1_PTR].blocation_s = location_s == 0 ? false : true;
							report_ghead_location_s(SERIAL1_PTR);
#else
							ghead_p[ptr].blocation_s = location_s == 0 ? false : true;
							report_ghead_location_s(ptr);
#endif	
						}
					}
					break;
				case CHECK_SENSOR_S_CMD:
					{
                        int k_s = 0;
						if((k_s = gcode_seen(ptr,'S')) != ERROR_VAL)
						{
#ifdef RMF500_TESTMODE
							ghead_p[SERIAL1_PTR].ksesor_s = k_s == 0 ? false : true ;
							report_ghead_ksensor_s(SERIAL1_PTR);
#else
							ghead_p[ptr].ksesor_s = k_s == 0 ? false : true ;
							report_ghead_ksensor_s(ptr);
#endif
						}					
					}
					break;
				case SET_FANSPEED_CMD:
					/* response from fan setting action */
					{
						int f_speed = 0;
						if((f_speed = gcode_seen(ptr,'S')) != ERROR_VAL)
						{
							ghead_p[ptr].fan_speed = f_speed;
							reponse_fanspeed_set_s(ptr);
						}
					}
					break;
				case PRO3_NORMAL_CMD:
				{
					int res = 0;
					int head_ptr = 0;
					switch(cmd_ans_ifo[port_ptr].cmd_ifo[0].sub_cmd)
					{
						case 'H':
							{
								head_ptr = cmd_ans_ifo[port_ptr].cmd_ifo[0].sub_val;
								switch(cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_cmd)
								{
									case 'T':
										if((res = gcode_seen_f(port_ptr,'T',&ghead_p[head_ptr].temp_f)) != ERROR_VAL)
										{
											ghead_p[head_ptr].temp_i = m_ftoi(ghead_p[head_ptr].temp_f,10);
											report_ghead_temp(head_ptr);
										}
										break;
									case 'A':
										repond_fuc(head_ptr,'A',cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_val);
										break;
									case 'H':
										repond_fuc(head_ptr,'H',cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_val);
										break;
									case 'K':
										ghead_p[head_ptr].ksesor_s = cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_val;
										//report_ghead_ksensor_s(head_ptr);
										repond_fuc(head_ptr,'K',ghead_p[head_ptr].ksesor_s);
										break;
									case 'V':
										send_len_data_to_host(0,'V',1,strlen((const char *)cmd_ans_ifo[port_ptr].cmd_ifo[1].val),cmd_ans_ifo[port_ptr].cmd_ifo[1].val);
										break;
								}
							}
							break;
						case 'T':

							repond_fuc(head_ptr,'D',cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_val);
							break;
						case 'S':
							repond_fuc(head_ptr,'W',cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_val);
							break;
						case 'V':
							send_len_data_to_host(0,'V'+'C',1,strlen((const char *)cmd_ans_ifo[port_ptr].cmd_ifo[0].val),cmd_ans_ifo[port_ptr].cmd_ifo[0].val);
							break;
						case 'L':
							break;
						case 'R':
							break;
						case 'A':
							break;

					}
					
				}
					break;
				case PRO3_HEATEND_RD_CMD:
				{
					uint8_t head_ptr = cmd_ans_ifo[port_ptr].cmd_ifo[0].sub_val;
					if(cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_cmd == 'A')
					{
						uint8_t addr = char2hex((const char *)cmd_ans_ifo[port_ptr].cmd_ifo[1].val,2);
						uint8_t mode = 'R' + cmd_ans_ifo[port_ptr].cmd_ifo[2].sub_cmd;
						send_len_data_to_host(head_ptr,mode,addr,strlen((const char *)cmd_ans_ifo[port_ptr].cmd_ifo[2].val),(uint8_t *)cmd_ans_ifo[port_ptr].cmd_ifo[1].val);
					}
				}
					break;
				case PRO3_HEATEND_RW_CMD:
				{
					uint8_t head_ptr = cmd_ans_ifo[port_ptr].cmd_ifo[0].sub_val;
					if(cmd_ans_ifo[port_ptr].cmd_ifo[1].sub_cmd == 'A')
					{
						uint8_t addr = char2hex((const char *)cmd_ans_ifo[port_ptr].cmd_ifo[1].val,2);
						uint8_t mode = 'W' + cmd_ans_ifo[port_ptr].cmd_ifo[2].sub_cmd;

						send_len_data_to_host(head_ptr,mode,addr,strlen((const char *)cmd_ans_ifo[port_ptr].cmd_ifo[1].val),(uint8_t *)cmd_ans_ifo[port_ptr].cmd_ifo[1].val);
					}
				}
					break;
				case PRO3_GHEAD_RESTART_EVENT:
					/* report to uplayer */
					repond_fuc(0,'R'+'H',0);
					/* check ghead version */
					mprintf(port_ptr,"M5100 CV\n");
					break;
				case BTEST_CMD:
					mprintf(DEBUG_PORT,"aatest = %d s = %s\r\n",12,"fuzhihao");
					{
						int m = 0;
						if((m = gcode_seen(ptr,'S')) != ERROR_VAL)
						{
							mprintf(DEBUG_PORT,"get v = %d\r\n",m);
						}
						if((m = gcode_seen(ptr,'G')) != ERROR_VAL)
						{
							mprintf(DEBUG_PORT,"start monitor ghead = %d\r\n",m);
							start_monitor_ghead(m == 0 ? SERIAL1_PTR : SERIAL2_PTR,500000);
						}
					}
					break;
				default:
					break;
			}	
		}
h_end:
		rec_cmd[ptr].cmd_handle_busy = false;
		cmd_reset_rec(ptr);
	}
}

static void cmd_reset_rec(uint8_t ptr)
{
	rec_cmd[ptr].cmd_rec_complete_flag = false;
	rec_cmd[ptr].cmd_rec_step = 0;
	rec_cmd[ptr].is_crc  = false;
	rec_cmd[ptr].crc_ptr = 0;
	rec_cmd[ptr].cmd_rec_ptr = 0;
	memset(rec_cmd[ptr].cmd,'\0',MAX_CMD_LEN);
	memset(rec_cmd[ptr].crc_buff,'\0',MAX_CRC_SIZE);
	/* new version */
	rec_cmd[ptr].cmd_val_ptr = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		rec_cmd[ptr].cmd_val[i].val_ptr  = 0;
		rec_cmd[ptr].cmd_val[i].val_type = 0;
		memset(rec_cmd[ptr].cmd_val[i].val,0,MAX_VAL_LEN);
	}
}

static void complete_rec(uint8_t ptr)
{
	rec_cmd[ptr].cmd_rec_complete_flag = true;
	rec_cmd[ptr].cmd_rec_step = 0;
	/* block normal temp sending */
	rec_cmd[ptr].cmd_handle_busy 	  = true;
	/* inform task to deal with cmd */
	
	if(ptr == SERIAL1_PTR)
	{
		send_msg(msg_box,SERIAL_RECV_CHN1_EVENT);
	}	
	else if(ptr == SERIAL2_PTR)
	{
		send_msg(msg_box,SERIAL_RECV_CHN2_EVENT);
	}
		
}

static void cmd_rec(uint8_t ptr,uint8_t dat)
{
	//mprintf(DEBUG_PORT,"%ds%d,%x\r\n",ptr,rec_cmd[ptr].cmd_rec_step,dat);
	switch(rec_cmd[ptr].cmd_rec_step)
	{
	case 0:
		if(rec_cmd[ptr].cmd_rec_complete_flag == false)
		{
			if(dat == 'M')
			{
				rec_cmd[ptr].cmd_ptr = 0;
				rec_cmd[ptr].cmd[rec_cmd[ptr].cmd_ptr++] = dat ;
				rec_cmd[ptr].cmd_rec_step = 1;

				rec_cmd[ptr].cmd_rec_ptr = 0;
				rec_cmd[ptr].cmd_rec_buff[rec_cmd[ptr].cmd_rec_ptr++] = dat ;

			}
		}
		break;
	case 1:
		/* wrong cmd, should discard it */
		if(rec_cmd[ptr].cmd_ptr >= MAX_CMD_LEN)
		{
			cmd_reset_rec(ptr);
			return ;
		}
		/* judge the rec bytes */
		if(dat != '\n' && dat <= 'Z' && dat != '\r' && dat != '\0' && dat != ' ')
		{
			rec_cmd[ptr].cmd[rec_cmd[ptr].cmd_ptr++] = dat ;

			rec_cmd[ptr].cmd_rec_buff[rec_cmd[ptr].cmd_rec_ptr++] = dat ;
		}
		else if(dat == ' ')
		{
			/* find mid space */
			rec_cmd[ptr].cmd[rec_cmd[ptr].cmd_ptr++] = '\0' ;
			rec_cmd[ptr].cmd_rec_step = 2;
			/* get first parameter */
			rec_cmd[ptr].cmd_val_ptr = 0;
			/* clear rec next para */
			rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr = 0;

			rec_cmd[ptr].cmd_rec_buff[rec_cmd[ptr].cmd_rec_ptr++] = dat ;
		}
		else if(dat == '\r' || dat == '\n' || dat == '\0')
		{
			/* no extra parameters just main cmd */
			rec_cmd[ptr].val_ptr = 0;
			complete_rec(ptr);
			return ;
		}
		else if(dat > 'Z')
		{
			/* get unexpected byte ,this message will be disgarded */
			cmd_reset_rec(ptr);
			return ;
		}
		break;
	case 2:
		/* normal s */
		if(dat >= 'A' && dat <= 'Z')
		{
//			rec_cmd[ptr].val_ptr = 0;
//			rec_cmd[ptr].val[rec_cmd[ptr].val_ptr++] = dat ;			
			rec_cmd[ptr].cmd_rec_step = 3;		
			rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_type = dat ;
			//rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr = 0;
			rec_cmd[ptr].cmd_rec_buff[rec_cmd[ptr].cmd_rec_ptr++] = dat ;
		}
		else if(dat == '\r' || dat == '\n' || dat == '\0')
		{
			/* get end description with no value */
			complete_rec(ptr);
			return ;
		}
		else
		{
			/* get unexpected byte ,this message will be disgarded */
			cmd_reset_rec(ptr);
			return ;
		}
		break;
	case 3:
		/* excceed scope, should discard it */
		if(rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr >= MAX_VAL_LEN)
		{
			cmd_reset_rec(ptr);
			return ;
		}
		if(dat == '*')
		{
			rec_cmd[ptr].cmd_rec_step = 4;
			rec_cmd[ptr].is_crc = true;
			rec_cmd[ptr].crc_ptr = 0;
		}
		else if(dat == '\r' || dat == '\n' || dat == '\0')
		{	
			rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val[rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr++] = '\0';
			rec_cmd[ptr].cmd_val_ptr++;
			complete_rec(ptr);
			return ;
		}
		else if(dat == ' ')
		{
			/* get extra space,following parameter comes */
			if(rec_cmd[ptr].cmd_val_ptr < MAX_CMD_N)
			{
				rec_cmd[ptr].cmd_val_ptr ++ ;
				rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr = 0;
				rec_cmd[ptr].cmd_rec_step = 2;

				rec_cmd[ptr].cmd_rec_buff[rec_cmd[ptr].cmd_rec_ptr++] = dat ;
			}
			else
			{
				/* can only rec 5 sub parameters */
				rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val[rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr++] = '\0';
				complete_rec(ptr);
			}	
		}
		else if(dat != '\n' && dat <= '~')
		{
			rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val[rec_cmd[ptr].cmd_val[rec_cmd[ptr].cmd_val_ptr].val_ptr++] = dat ;

			rec_cmd[ptr].cmd_rec_buff[rec_cmd[ptr].cmd_rec_ptr++] = dat ;
		}
		else
		{
			cmd_reset_rec(ptr);
			return ;
		}
		break;
	case 4:
		if(dat >= '0' && dat <= '9')
		{
			rec_cmd[ptr].crc_buff[rec_cmd[ptr].crc_ptr++] = dat;
		}
		else if(dat == '\0' || dat == '\r' || dat == '\n' || rec_cmd[ptr].crc_ptr >=3)
		{
			rec_cmd[ptr].crc_buff[rec_cmd[ptr].crc_ptr++] = '\0';
			complete_rec(ptr);
		}
		break;
	default:
		rec_cmd[ptr].cmd_rec_step = 0;
		break;
	}
}







