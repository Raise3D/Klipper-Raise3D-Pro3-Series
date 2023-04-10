#include "stdio_t.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "generic/serial_cdc.h"


#ifndef uint8_t
#define uint8_t unsigned char 
#define uint16_t unsigned short 
#define uint32_t unsigned int 
#endif


#ifndef NULL
#define NULL
#endif

static void recv_len_dat(uint8_t *buff,uint16_t len);

#define MAX_SEND_LEN 256
#define MAX_STR_LEN 128
static char send_buff[MAX_SEND_LEN];


static _nor_serial_op nor_serial_op[2] =
{
	{
		.send_len_dat    =    NULL,
		.recv_len_dat     =   recv_len_dat ,
		.notify                   =  NULL,
	},
	{
		.send_len_dat    =    NULL,
		.recv_len_dat     =   recv_len_dat ,
		.notify                   =  NULL,
	}
};
static void recv_len_dat(uint8_t *buff,uint16_t len)
{
	/*  */	


}
static void send_len_dat(uint8_t ptr,uint8_t *buff,uint16_t len)
{
	if(nor_serial_op[ptr].send_len_dat != NULL)
	{
		nor_serial_op[ptr].send_len_dat(buff,len);
	}
}
_nor_serial_op *register_io_serial(uint8_t ptr,void (*send_fuc)(uint8_t *,uint16_t))
{
	nor_serial_op[ptr].send_len_dat = send_fuc;
	return &nor_serial_op[ptr];
}


static int myitoa(int n, char str[], int radix)  
{  
	int len, i , j , remain;  
	char tmp,f = 0;  
	len = i = 0;  
	if(n < 0)
	{
		n = 0 - n;
		f = 1;
	}
	do
	{  
		remain = n % radix;  
		if(remain > 9)  
			str[i] = remain  - 10 + 'A';  
		else  
			str[i] = remain + '0';
		i++;  
	}while(n /= radix);
	str[i] = '\0';
	len = i;
	for(i-- , j = 0 ; j <= i ; j++ , i--) 
	{  
		tmp = str[j];  
		str[j] = str[i];  
		str[i] = tmp;  
	}
	if(f == 1)
	{
		for(i = 0; i < len ;i ++)
		{
			str[len - i] = str[len -i - 1];
		}
		str[0] = '-';
	}
	
	return len;
}  
static uint8_t copy_str(char *dst,char *src)
{
	uint8_t i = 0;
	while(i  <  MAX_STR_LEN)
	{
		if(src[i] != '\0')
		{
			dst[i] = src[i];
			i++;
		}
		else
		{
			dst[i] = src[i];
			return 1;
		}
	}
	return 1;
}
static char *find_str_end(char *dst)
{
	uint8_t i = 0;
	while(i < 128)
	{
		if(*dst == '\0')
		{
			return dst;
		}
		dst++;
		i++;
	}
	return NULL;
}
static uint8_t strcat_t(char *dst,char *src)
{
	char *cd  = find_str_end(dst);
	if(cd != NULL)
	{
		return copy_str(cd,src);
	}
	return 0;
}

static void memset_t(uint8_t *dat,char c,uint8_t n)
{
	for(uint8_t i = 0; i < n ; i++)
	{
		dat[i] = (uint8_t)c;
	}
}
static void putchar_t(char c)
{
	char *cd  = find_str_end(send_buff);
	*cd= c ;
	//*cd = '\0';
}
static char str[50];

void mprintf(uint8_t ptr,char *fmt,...)
{
	uint8_t i,len = strlen(fmt);
	uint8_t res = 0;
	//char str[] = "\r\n";
	memset_t((uint8_t *)send_buff,'\0',MAX_SEND_LEN-1);
	memset_t((uint8_t *)str,'\0',30);
#ifdef CONFIG_SPORT_B_NONE_DEBUG
	return;
#endif
	if(len == 0 || len > 128)
	{
			return ;
	}
	for(i = 0 ; i < len ; i++)
	{
			if(fmt[i] == '%')
			{
					res = 1;
					break;
			}
	}
	if(res == 0)
	{
			send_len_dat(ptr,( uint8_t *)fmt,len);
	}
	else
	{
			char c,ch;
			va_list args;
			int pos = 0;
			va_start(args, fmt);
			//n = vsprintf((char *)send_buff, fmt, args);
			while(pos < 128)  
		          {  
			       c = fmt[pos++] ;
			        switch(c)  
			        {  
			        case '\0':
				  goto end;
			        case '%':
			            ch = fmt[pos++];  
			            switch(ch)  
			            {  
			            case 'd':
					 {  
			                    uint32_t n = va_arg(args, uint32_t);
			                    myitoa(n, str, 10);  
			                    strcat_t(send_buff,str);
			                    break;  
			                }  
			            case 'x':
				       	{  
			                    int n = va_arg(args, int);  
			                    myitoa(n, str, 16);  
			                    strcat_t(send_buff,str); 
			                    break;  
			                }  
			            case 'f':  
			                {  
			                    double f = va_arg(args, double);
			                    int n;  
			                    n = f;  
			                    myitoa(n, str, 10);
					    		strcat_t(send_buff,str);  
			                    putchar_t('.');  
			                    n = (f - n) * 100;
								memset_t((uint8_t *)str,'\0',30);  
			                    myitoa(n, str, 10);  
			                    strcat_t(send_buff,str);   
			                    break;  
			                }  
			            case 'c':  
			                {  
			                    putchar_t(va_arg(args, int));  
			                    break;  
			                }  
			            case 's':  
			                {  
			                    char *p = va_arg(args, char *); 
					    strcat_t(send_buff,p); 
			                    break;  
			                }  
			            case '%':
			                {  
			                    putchar_t('%');  
			                    break;  
			                }  
				 case '\0':
				      goto end;
			            default:  
			                {   
					putchar_t(c);
			                     break;;  
			                }  
			            }  
			            break;  
			        default:  
						putchar_t(c);
			            break;  
			        }  
			    }  		
end:			
	        		va_end(args);
			send_len_dat(ptr,( uint8_t *)send_buff,(uint16_t)strlen((const char*)send_buff));
	}
}




