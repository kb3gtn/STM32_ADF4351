#include <stdint.h>


///////////////////////// UINT8_T RING BUFFER //////////////////////////////
typedef struct ringbuf_uint8_t
{
    uint8_t *buf;                    //points to data array
    unsigned char length;            //length of data array
    unsigned char head, tail;        //producer and consumer indices
} ringbuf_uint8t;


//initializes the given ringbuffer with the supplied array and its length
extern void rb_init(ringbuf_uint8t *rb, uint8_t *array, unsigned char length);

//returns boolean true if the ringbuffer is empty, false otherwise
extern unsigned char rb_isempty(ringbuf_uint8t *rb);

//returns boolean true if the ringbuffer is full, false otherwise
extern unsigned char rb_isfull(ringbuf_uint8t *rb);

//consumes an element from the buffer
//returns NULL if buffer is empty or a pointer to the array element otherwise
extern int rb_get(ringbuf_uint8t *rb, uint8_t *data);

//puts an element into the buffer
//returns 0 if buffer is full, otherwise returns 1
extern unsigned char rb_put(ringbuf_uint8t *rb, uint8_t c);

