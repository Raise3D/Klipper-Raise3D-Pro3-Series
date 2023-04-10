// sam3/sam4 serial port
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_data
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // gpio_peripheral
#include "sched.h" // DECL_INIT
#include "generic/def.h"
#include "generic/serial_cdc.h"
#include "generic/irq.h"
#include "stdio_t.h"


#if (CONFIG_MACH_SAM4E == 1)


void UART0_Handler(void);
void UART1_Handler(void);


enum _UART_STATUS
{
	UART_SEND_IDLE = 0,
	UART_SEND_BUSY,
	UART_SEND_BUF_NOT_EMPTY,
	UART_SEND_BUF_EMPTY,
};


#define MAX_SEND_LEN 512
typedef struct _UART_PARA
{
	 Uart*   Port ;
	uint32_t rx_pin;
	uint32_t tx_pin;
	uint8_t sendbuff[MAX_SEND_LEN];
	uint16_t send_put_ptr;
	uint16_t send_pop_ptr;
	uint16_t send_len;
	uint8_t send_status;
	char pin_pt;
	uint32_t Pmc_id;
}_uart_para;

_uart_para uart_para[2];
#define DEFAULT_BAUD_RATE 115200

#define CHECK_UART(PTR)  (PTR <= SERIAL2_PTR && uart_para[PTR].Port  != NULL)

//#define CHECK_UART(PTR)  (1)

static void try_send(uint8_t ptr);
static void serial1_send_dat(uint8_t *buff,uint16_t len);
static void serial2_send_dat(uint8_t *buff,uint16_t len);


#define ENTER_CRITICAL()     irq_disable()
#define EXIT_CRITCAL()		irq_enable()

_nor_serial_op *serial_op[2] = {NULL};


void serial_init_t(uint8_t ptr,uint32_t baud)
{
	if(CHECK_UART(ptr))
	{
		Uart* port = uart_para[ptr].Port ;
		gpio_peripheral(uart_para[ptr].rx_pin, uart_para[ptr].pin_pt, 1);
		gpio_peripheral(uart_para[ptr].tx_pin, uart_para[ptr].pin_pt, 1);

		// Reset uart
		enable_pclock(uart_para[ptr].Pmc_id);
		port->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
		port->UART_CR = (UART_CR_RSTRX | UART_CR_RSTTX
		                 | UART_CR_RXDIS | UART_CR_TXDIS);
		port->UART_IDR = 0xFFFFFFFF;

		// Enable uart
		port->UART_MR = (US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO
		                 | UART_MR_CHMODE_NORMAL);
#ifdef CONFIG_GH_BUAD_RATE 
		port->UART_BRGR = SystemCoreClock / (16 * baud);
#else
		port->UART_BRGR = SystemCoreClock / (16 * DEFAULT_BAUD_RATE);
#endif
		port->UART_IER = UART_IER_RXRDY;

		if(ptr == SERIAL1_PTR)
			armcm_enable_irq(UART0_Handler, UART0_IRQn, 0);
		else
			armcm_enable_irq(UART1_Handler, UART1_IRQn, 0);
		port->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

		serial_op[ptr] = register_gcode_serial(ptr,ptr == SERIAL1_PTR ? serial1_send_dat : serial2_send_dat);
		register_io_serial(ptr,ptr == SERIAL1_PTR ? serial1_send_dat : serial2_send_dat);
		
	}
	

}
static uint8_t send_dat_in_queue(uint8_t ptr,uint8_t *buff,uint8_t len)
{
	if(CHECK_UART(ptr))
	{
		if((uart_para[ptr].send_len + len )< MAX_SEND_LEN )
		{
			uint8_t i = 0;
			/* load dat to be sent */
			
			/*enter critical */
			ENTER_CRITICAL();
			for(i = 0 ; i < len ; i ++)
			{
				uart_para[ptr].sendbuff[uart_para[ptr].send_put_ptr] = buff[i] ;
				uart_para[ptr].send_put_ptr = (uart_para[ptr].send_put_ptr + 1)%MAX_SEND_LEN;
				uart_para[ptr].send_len++;
			}
			/* exit critical */
			EXIT_CRITCAL();
			/* try to send */
			if(uart_para[ptr].send_status == UART_SEND_IDLE)
			{
				try_send(ptr);
			}
			return true;
		}
	}
	return false;
}
static void serial1_send_dat(uint8_t *buff,uint16_t len)
{
	send_dat_in_queue(SERIAL1_PTR,buff,len);
}
static void serial2_send_dat(uint8_t *buff,uint16_t len)
{
	send_dat_in_queue(SERIAL2_PTR,buff,len);
}

static uint8_t pop_dat_from_queue(uint8_t ptr,uint8_t *dat)
{
	if(CHECK_UART(ptr))
	{
		if(uart_para[ptr].send_len > 0)
		{
			*dat = uart_para[ptr].sendbuff[uart_para[ptr].send_pop_ptr];
			uart_para[ptr].send_pop_ptr = (uart_para[ptr].send_pop_ptr + 1)%MAX_SEND_LEN;
			uart_para[ptr].send_len--;
			return UART_SEND_BUF_NOT_EMPTY;
		}
	}
	return UART_SEND_BUF_EMPTY;
}
static void try_send(uint8_t ptr)
{
	if(CHECK_UART(ptr))
	{
		uint8_t dat ;
		if(pop_dat_from_queue(ptr,&dat) != UART_SEND_BUF_EMPTY)
		{
			uart_para[ptr].Port->UART_THR = dat ;
			if(uart_para[ptr].send_status == UART_SEND_IDLE)
			{
				uart_para[ptr].Port->UART_IER = UART_IDR_TXRDY;
				uart_para[ptr].send_status = UART_SEND_BUSY;
			}
		}
		else
		{
			uart_para[ptr].Port->UART_IDR = UART_IDR_TXRDY;
			uart_para[ptr].send_status = UART_SEND_IDLE;
		}
	}
}
static void rec_dat(uint8_t ptr,uint8_t dat)
{
	if(CHECK_UART(ptr))
	{
		/* test */
		uint8_t data = dat;
		if(serial_op[ptr] != NULL && serial_op[ptr] ->recv_len_dat != NULL)
		{
			serial_op[ptr]->recv_len_dat(&data,1);
		}
	}
}
static void uart_interrupt_callback(uint8_t ptr)
{
	if(CHECK_UART(ptr))
	{
		uint32_t status = uart_para[ptr].Port->UART_SR;
		if (status & UART_SR_RXRDY)
		{
			rec_dat(ptr,uart_para[ptr].Port->UART_RHR);	  
			//uart_para[SERIAL2_PTR].Port->UART_THR = uart_para[ptr].Port->UART_RHR;
		}
		if (status & UART_SR_TXRDY)
		{
			try_send(ptr);
		}
	}
}


#endif

void init_uart_para(void)
{
#if (CONFIG_ATSAM_SINGLE_GHEAD_L == 1)
	uart_para[SERIAL1_PTR].Port = UART0;
	uart_para[SERIAL1_PTR].rx_pin = GPIO('A', 9);
	uart_para[SERIAL1_PTR].tx_pin = GPIO('A', 10);

	uart_para[SERIAL2_PTR].Port = NULL;
#elif (CONFIG_ATSAM_SINGLE_GHEAD_R == 1)
	uart_para[SERIAL2_PTR].Port = UART1;
	uart_para[SERIAL2_PTR].rx_pin = GPIO('A', 5);
	uart_para[SERIAL2_PTR].tx_pin = GPIO('A', 6);

	uart_para[SERIAL1_PTR].Port = NULL;
#elif (CONFIG_ATSAM_DOUBLE_GHEAD == 1)
	uart_para[SERIAL1_PTR].Port = UART0;
	uart_para[SERIAL1_PTR].rx_pin = GPIO('A', 9);
	uart_para[SERIAL1_PTR].tx_pin = GPIO('A', 10);

	uart_para[SERIAL2_PTR].Port = UART1;
	uart_para[SERIAL2_PTR].rx_pin = GPIO('A', 5);
	uart_para[SERIAL2_PTR].tx_pin = GPIO('A', 6);
#else
	uart_para[SERIAL1_PTR].Port = UART0;
	uart_para[SERIAL1_PTR].rx_pin = GPIO('A', 9);
	uart_para[SERIAL1_PTR].tx_pin = GPIO('A', 10);

	uart_para[SERIAL2_PTR].Port = UART1;
	uart_para[SERIAL2_PTR].rx_pin = GPIO('A', 5);
	uart_para[SERIAL2_PTR].tx_pin = GPIO('A', 6);
#endif
	uart_para[SERIAL1_PTR].send_pop_ptr = uart_para[SERIAL1_PTR].send_put_ptr = uart_para[SERIAL1_PTR].send_len = 0;
	uart_para[SERIAL2_PTR].send_pop_ptr = uart_para[SERIAL2_PTR].send_put_ptr = uart_para[SERIAL2_PTR].send_len = 0;
	uart_para[SERIAL1_PTR].send_status = UART_SEND_IDLE;
	uart_para[SERIAL2_PTR].send_status = UART_SEND_IDLE;

	uart_para[SERIAL1_PTR].Pmc_id = ID_UART0 ;
	uart_para[SERIAL2_PTR].Pmc_id = ID_UART1 ;

	uart_para[SERIAL1_PTR].pin_pt = 'A';
	uart_para[SERIAL2_PTR].pin_pt = 'C';
}
// Serial port pins
#if CONFIG_MACH_SAM3X
#define UARTx_IRQn UART_IRQn
static Uart * const Port = UART;
static const uint32_t Pmc_id = ID_UART;
static const uint32_t rx_pin = GPIO('A', 8), tx_pin = GPIO('A', 9);
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA8,PA9");
#elif CONFIG_MACH_SAM4S
#define UARTx_IRQn UART1_IRQn
static Uart * const Port = UART1;
static const uint32_t Pmc_id = ID_UART1;
static const uint32_t rx_pin = GPIO('B', 2), tx_pin = GPIO('B', 3);
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB2,PB3");
#elif CONFIG_MACH_SAM4E

#if CONFIG_ATSAM_SINGLE_GHEAD_L
DECL_CONSTANT_STR("RESERVE_PINS_serial1", "PA9,PA10");
#elif CONFIG_ATSAM_SINGLE_GHEAD_R
DECL_CONSTANT_STR("RESERVE_PINS_serial2", "PA5,PA6");
#elif CONFIG_ATSAM_DOUBLE_GHEAD
DECL_CONSTANT_STR("RESERVE_PINS_serial1", "PA9,PA10");
DECL_CONSTANT_STR("RESERVE_PINS_serial2", "PA5,PA6");
#endif

#endif

#if (CONFIG_MACH_SAM4E == 1)

void UART0_Handler(void)
{
    uart_interrupt_callback(SERIAL1_PTR);
}
void UART1_Handler(void)
{
    uart_interrupt_callback(SERIAL2_PTR);
}
#else
void
UARTx_Handler(void)
{
    uint32_t status = Port->UART_SR;
    if (status & UART_SR_RXRDY)
     {

     }
        //serial_rx_byte(Port->UART_RHR);
    if (status & UART_SR_TXRDY) {
    {
	uint8_t  data , ret = 1;
        if (ret)
            Port->UART_IDR = UART_IDR_TXRDY;
        else
            Port->UART_THR = data;
     }	
    }
}

#endif


void
serial_enable_tx_irq(void)
{
#if (CONFIG_MACH_SAM4E == 1)

#else
    Port->UART_IER = UART_IDR_TXRDY;
#endif
}



void
serial_init(void)
{

#if (CONFIG_MACH_SAM4E == 1)
	init_uart_para();
	serial_init_t(SERIAL1_PTR,115200);
	serial_init_t(SERIAL2_PTR,230400);
#else
	gpio_peripheral(rx_pin, 'A', 1);
	gpio_peripheral(tx_pin, 'A', 0);

	// Reset uart
	enable_pclock(Pmc_id);
	Port->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	Port->UART_CR = (UART_CR_RSTRX | UART_CR_RSTTX
	                 | UART_CR_RXDIS | UART_CR_TXDIS);
	Port->UART_IDR = 0xFFFFFFFF;

	// Enable uart
	Port->UART_MR = (US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO
	                 | UART_MR_CHMODE_NORMAL);
#if CONFIG_SERIAL_BAUD
	Port->UART_BRGR = SystemCoreClock / (16 * CONFIG_SERIAL_BAUD);
#elif CONFIG_GH_BUAD_RATE
	Port->UART_BRGR = SystemCoreClock / (16 * CONFIG_GH_BUAD_RATE);
#endif
	Port->UART_IER = UART_IER_RXRDY;
	armcm_enable_irq(UARTx_Handler, UARTx_IRQn, 0);
	Port->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
#endif

   
}
DECL_INIT(serial_init);
