#include "base/PipeBufferv2.h"
#include "base/Component.h"

using namespace PCMCsim;

PipeBufferv2::PipeBufferv2( ): 
prgm_ready(true), prgm_stall(false), input(NULL), 
nxt_pb(NULL), size(1), front(-1), rear(-1)
{
    buffer = new pbe_t[size];
    memset(buffer, 0, size*sizeof(pbe_t));
}

PipeBufferv2::PipeBufferv2(int latency): 
prgm_ready(true), prgm_stall(false), input(NULL), 
nxt_pb(NULL), size(latency), front(-1), rear(-1)
{
    buffer = new pbe_t[size];
    memset(buffer, 0, size*sizeof(pbe_t));
}

PipeBufferv2::~PipeBufferv2( )
{
    // TODO: deallocate signals at module first
    delete[] buffer;

    size = 0;
    front = -1;
    rear = -1;
}

void* PipeBufferv2::proceed( )
{
    /* Proceed stage signals */
    void* ret = NULL;
    int idx = front;
    while (idx >= 0)
    {
        if (canProceed(idx))
            buffer[idx].stage += 1;
        if (idx == rear)
            break;
        
        idx = (idx+1)%size;
    }
    
    /* 
     * Push input & pop front 
     * Operable if next pipe buffer (or module)
     * can accept the output of this pipe buffer
     */
    if (front>=0 && 
        (nxt_pb==NULL ||            // last pipe in last module
         nxt_pb->isAcceptable( )))  // next pipe can accept signals
    {
        /* Pop out front to next pipe/module */
        if (buffer[front].stage==size && prgm_stall==false)
        {
            if (nxt_pb)
                nxt_pb->input = buffer[front].signals;
            else
                ret = buffer[front].signals;

            dequeue( );
        }
    }
    
    if (input)
    {
        enqueue(input);
        input = NULL;
    }

    return ret;
}

bool PipeBufferv2::canProceed(int idx)
{
    /* Check whether the current entry can proceed to next stage */
    assert(idx >= 0);
    assert((front<=rear && idx>=front && idx<=rear) ||
           (front>rear && (idx>=front || idx <=rear)));

    if (idx == front)
    {
        if (buffer[front].stage < size)
            return true;
    }
    else 
    {
        int prev = -1;
        if (idx == 0)
            prev = size - 1;
        else
            prev = idx - 1;
        assert(prev >= 0);
   
        int diff_stage = buffer[prev].stage - buffer[idx].stage; 
        if (diff_stage > 1)
            return true;
        else if (diff_stage <= 0)
            assert(0);
    }

    return false;
}

bool PipeBufferv2::isAcceptable( )
{
    /* Check whether a new packet can be accepted by this pipe */
    bool ret = false;

    if (prgm_ready)
    {
        if (nxt_pb)
        {
            if (nxt_pb->prgm_stall)
            {
                if (isFull( ))
                    ret = false;
                else
                    ret = true;
            }
            else
            {
                if (nxt_pb->isFull( ))
                {
                    if (nxt_pb->isAcceptable( ))
                        ret = true;
                    else
                        ret = false;
                }
                else
                    ret = true;
            }
        }
        else if (prgm_stall)
        {
            if (isFull())
                ret = false;
            else
                ret = true;
        }
        else
            ret = true;
    }
    else
        ret = false;

    return ret;
}

int PipeBufferv2::getSize( )
{
    return size;
}

int PipeBufferv2::getHeadIdx( )
{
    return front;
}

int PipeBufferv2::getRearIdx( )
{
    return rear;
}

bool PipeBufferv2::isFull( )
{
    if (isEmpty( ))
        return false;
    else if ((front == 0 && rear == size-1) 
        || (rear == (front-1)%(size-1)))
        return true;

    return false;
}

bool PipeBufferv2::isEmpty( )
{
    return (front == -1);
}

void PipeBufferv2::enqueue(void* signals)
{
    assert(isFull( )==false);

    if (front == -1)
    {
        /* 1st element insertion */
        front = 0;
        rear = 0;
    }
    else if (rear == size-1)
    {
        assert(front > 0);
        rear = 0;
    }
    else
        rear += 1;

    buffer[rear].signals = signals;
    buffer[rear].stage = 0;
}

void* PipeBufferv2::dequeue_rear( )
{
    assert(rear>=0);
    assert(isEmpty( )==false);

    void* ret = buffer[rear].signals;
    buffer[rear].signals = NULL;
    buffer[rear].stage = 0;

    if (rear == front)
    {
        /* Last element deletion */
        front = -1;
        rear = -1;
    }
    else if (rear == 0)
        rear = size-1;
    else
        rear -= 1;

    return ret;
}

void* PipeBufferv2::dequeue( )
{
    assert(front>=0);
    assert(isEmpty( )==false);

    void* ret = buffer[front].signals;
    buffer[front].signals = NULL;
    buffer[front].stage = 0;
    if (front == rear)
    {
        /* The popped one is the last one */
        front = -1;
        rear = -1;
    }
    else if (front == size-1)
        front = 0;
    else
        front += 1;

    return ret;
}



