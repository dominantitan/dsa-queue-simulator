#include traffic.h

void initQueue(VehicleQueue *q){
    q->front = -1;
    q->rear = -1;
    q->count = 0;
}

void isFull(VehicleQueue *q){
    return (q->count >= MAX_QUEUE_SIZE);
}

void isEmpty(VehicleQueue *q){
    return (q->count == 0);
}

void enqueue(){

}

void dequeue(){
    
}