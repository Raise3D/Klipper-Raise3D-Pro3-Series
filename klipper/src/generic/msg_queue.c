#include "msg_queue.h"

#define MAX_MSG_BOX_N    5
_msg_queue msg_q_list[MAX_MSG_BOX_N] = {0};


_msg_queue *lookup_msg_q_list(void)
{
    for(int i = 0;i < MAX_MSG_BOX_N ; i++)
    {
        if(msg_q_list[i].is_used == false)
        {
            return &msg_q_list[i];
        }
    }
    return NULL;
}
void init_msg_list(void)
{
    for(int i = 0;i < MAX_MSG_BOX_N ; i++)
    {
        msg_q_list[i].is_used = false;
    }

}
int free_msg_box(_msg_queue *msg_box)
{
    if(msg_box != NULL)
    {
        msg_box->is_used = false;
        return 0;
    }
    return -1;
}
_msg_queue *create_msg_box(void (*wake_task)(void))
{
    _msg_queue *msg_q = lookup_msg_q_list();
    if(msg_q != NULL)
    {
        msg_q->msg_get_ptr = msg_q->msg_put_ptr = msg_q->mlen = 0;
        msg_q->wake_task = wake_task;
        return msg_q; 
    }
    return NULL;
}
int read_msg(_msg_queue *msg_box)
{
    if(msg_box != NULL)
    {
        if(msg_box->mlen > 0)
        {
            unsigned char res = msg_box->msg_box[msg_box->msg_get_ptr];
            msg_box->msg_get_ptr = (msg_box->msg_get_ptr + 1) % msg_box->msg_n;
            msg_box->mlen -- ;
            if(msg_box->mlen > 0)
            {
                if(msg_box->wake_task)
                {
                    msg_box->wake_task();
                }
            }
            return res;
        }
    }
    return MSG_ERROR;
}
int send_msg(_msg_queue *msg_box,unsigned char msg)
{
    if(msg_box != NULL)
    {
        if(msg_box->mlen < MSG_BOX_SIZE)
        {
            msg_box->msg_box[msg_box->msg_put_ptr] = msg ;
            msg_box->msg_put_ptr = (msg_box->msg_put_ptr + 1) % msg_box->msg_n;
            msg_box->mlen ++ ;
            if(msg_box->wake_task)
            {
                msg_box->wake_task();
            }
        }
        return 0;
    }
    return MSG_ERROR;
}
