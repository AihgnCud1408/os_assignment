#include <stdio.h>
#include <stdlib.h>
#include "queue.h"

int empty(struct queue_t *q)
{
        if (q == NULL)
                return 1;
        return (q->size == 0);
}

void enqueue(struct queue_t *q, struct pcb_t *proc)
{
        /* TODO: put a new process to queue [q] */
        if (q->size == MAX_QUEUE_SIZE)
                return;
        q->proc[q->size++] = proc;
}

struct pcb_t *dequeue(struct queue_t *q)
{
        /* TODO: return a pcb whose priority is the highest
         * in the queue [q] and remember to remove it from q
         * */
        if (empty(q))
        {
                return NULL;
        }
        struct pcb_t *proc;
#ifdef MLQ_SCHED
        int highest_priority = q->proc[0]->priority;
        int highest_priority_index = 0;
        for (int i = 1; i < q->size; i++)
        {
                if (q->proc[i]->priority < highest_priority)
                {
                        highest_priority = q->proc[i]->priority;
                        highest_priority_index = i;
                }
        }
        proc = q->proc[highest_priority_index];
        for (int i = highest_priority_index; i < q->size - 1; i++)
        {
                q->proc[i] = q->proc[i + 1];
        }
        q->size--;
        return proc;
#else
        proc = q->proc[0];
        for (int i = 1; i < q->size; i++)
        {
                q->proc[i - 1] = q->proc[i];
        }
        q->size--;
        return proc;
#endif
}
