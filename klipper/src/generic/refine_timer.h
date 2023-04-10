#ifndef _REFINE_TIMER_H_
#define _REFINE_TIMER_H_

#include <stdint.h>
/*timer**/
#ifndef NULL
#define NULL 0
#endif

#ifndef _TIMER_STRUCT_
#define _TIMER_STRUCT_
typedef void* timer_handler;
typedef enum TIMER_STATE
{
    TIMER_UNUNSED,
    TIMER_STOPPED,
    TIMER_RUNNING,
    TIMER_COMPLETED,
}TIMER_STATE;

typedef enum TIMER_TYPE
{
    TIMER_PERIOD,
    TIMER_ONESHOT
}TIMER_TYPE;
struct timer_struct
{
//	struct list_head entry;
	uint32_t		tcount;
	uint32_t		period;
	void	        (*callback)(timer_handler,void*);
	void            *private_data;
	TIMER_STATE		state;
	TIMER_TYPE		type;
};
#endif

void init_refine_timer_list(void);
/*timer*/
void refine_timer_sys_run(void);
TIMER_STATE refine_timer_status(timer_handler handler);
void refine_timer_destroy(timer_handler handler);
void refine_timer_stop(timer_handler handler);
void refine_timer_reset(timer_handler handler,uint32_t period);
void refine_timer_start(timer_handler handler);
timer_handler refine_timer_create(uint32_t period,TIMER_TYPE type,void (*cb)(timer_handler,void*),void *private_data);

#endif
