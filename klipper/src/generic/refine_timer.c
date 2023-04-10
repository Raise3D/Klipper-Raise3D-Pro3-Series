#include "refine_timer.h"
#include "board/irq.h" 

#define disable_irq() irq_disable()
#define enable_irq() irq_enable()

#define  TIMER_BASE_COUNT 10
/**/
#define REFINE_TIMER_MAX_NUMBER   5

static struct timer_struct refine_timer_table[REFINE_TIMER_MAX_NUMBER] = {0};

/**/
void init_refine_timer_list(void)
{
  uint8_t timerptr = 0;
  for(timerptr = 0; timerptr < REFINE_TIMER_MAX_NUMBER ;timerptr ++)
	refine_timer_table[timerptr].state = TIMER_UNUNSED;
}
/* timer */
timer_handler refine_timer_create(uint32_t period,TIMER_TYPE type,
                        void (*cb)(timer_handler,void*),void *private_data)
{
	uint32_t i;
	struct timer_struct *ptimer;

	if(period == 0)
	{
        return NULL;
	}

	disable_irq();
	for(i = 0;i < REFINE_TIMER_MAX_NUMBER;i++)
	{
		ptimer = &refine_timer_table[i];
		if(ptimer->state == TIMER_UNUNSED)
		{
                  ptimer->state = TIMER_STOPPED;
                  ptimer->type = type;
                  ptimer->period = period*TIMER_BASE_COUNT;
                  ptimer->tcount = 0;
                  ptimer->callback = cb;
                  ptimer->private_data = private_data;
                  break;
		}
	}
	enable_irq();

    if(i == REFINE_TIMER_MAX_NUMBER)
    {
        return NULL;
    }
    return (timer_handler)ptimer;
}

void refine_timer_destroy(timer_handler handler)
{
    struct timer_struct *ptimer = (struct timer_struct*)handler;

	if(!ptimer)
	{
		return;
	}

	disable_irq();
	if(ptimer->state != TIMER_UNUNSED)
	{
		//list_del_init(&ptimer->entry);

	}
	ptimer->state = TIMER_UNUNSED;
        handler = NULL;
	enable_irq();
}

void refine_timer_start(timer_handler handler)
{
    struct timer_struct *ptimer = (struct timer_struct*)handler;

	if(!ptimer)
	{
		return;
	}
	disable_irq();
	if(ptimer->state == TIMER_RUNNING)
	{
		enable_irq();
		return;
	}
	ptimer->state = TIMER_RUNNING;
	enable_irq();
}
void refine_timer_reset(timer_handler handler,uint32_t period)
{
	struct timer_struct *ptimer = (struct timer_struct*)handler;

	if(!ptimer || !period)
	{
		return;
	}
	disable_irq();
  if(ptimer->state == TIMER_RUNNING || ptimer->state == TIMER_COMPLETED)
  {
   ptimer->period = period*TIMER_BASE_COUNT;
   ptimer->tcount = 0;
  }
	enable_irq();
}
void refine_timer_stop(timer_handler handler)
{
    struct timer_struct *ptimer = (struct timer_struct*)handler;

	if(!ptimer)
	{
		return;
	}

	disable_irq();
	if(ptimer->state == TIMER_RUNNING || ptimer->state == TIMER_COMPLETED)
	ptimer->state = TIMER_STOPPED;
	enable_irq();
}
TIMER_STATE refine_timer_status(timer_handler handler)
{
	struct timer_struct *ptimer = (struct timer_struct*)handler;

	if(!ptimer)
	{
		return TIMER_UNUNSED;
	}

	return ptimer->state;
}
void refine_timer_sys_run(void)
{
   uint8_t timer_ptr = 0;
   for(timer_ptr = 0;timer_ptr < REFINE_TIMER_MAX_NUMBER ; timer_ptr++)
   {
     if(refine_timer_table[timer_ptr].state == TIMER_RUNNING)
     {
        refine_timer_table[timer_ptr].tcount ++ ;
        if(refine_timer_table[timer_ptr].tcount >= refine_timer_table[timer_ptr].period)
        {
           refine_timer_table[timer_ptr].tcount = 0;
           if(refine_timer_table[timer_ptr].type == TIMER_ONESHOT)
           {
              refine_timer_table[timer_ptr].state = TIMER_COMPLETED ;
           }
           if(refine_timer_table[timer_ptr].callback != NULL)
           refine_timer_table[timer_ptr].callback((timer_handler)&refine_timer_table[timer_ptr],refine_timer_table[timer_ptr].private_data);
        }
     }
   }
}


/*********/
