#ifndef __MSGQUEUE_H__
#define __MSGQUEUE_H__

#ifndef NULL
#define NULL 0
#endif
#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif

#define MSG_BOX_SIZE 16

#define  MSG_OK     0
#define  MSG_ERROR -1

typedef struct _MSG_QUEUE
{
    unsigned char msg_box[MSG_BOX_SIZE] ;
    unsigned char msg_n;
    unsigned char msg_get_ptr;
    unsigned char msg_put_ptr;
    unsigned char mlen;
    unsigned char is_used ;
    void (*wake_task)(void);
    /* data */
}_msg_queue;

//_msg_queue *create_msg_box(void (*wake_task)(void),unsigned char n);
_msg_queue *create_msg_box(void (*wake_task)(void));
int free_msg_box(_msg_queue *msg_box);
int send_msg(_msg_queue *msg_box,unsigned char msg);
int read_msg(_msg_queue *msg_box);
void init_msg_list(void);
#endif