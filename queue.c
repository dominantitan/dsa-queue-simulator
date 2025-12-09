#include traffic.h

//Queue initialization
void initQueue(VehicleQueue *q){
    q->front = 0;
    q->rear = -1;
    q->count = 0;
}

//To check whether the queue is full
void isFull(VehicleQueue *q){
    return (q->count >= MAX_QUEUE_SIZE);
}

//To check whether the queue is empty
void isEmpty(VehicleQueue *q){
    return (q->count == 0);
}

//
void enqueue(VehicleQueue *q,Vehicle v){
    if(isFull()){
        return;
    }
    q->(rear+1)%MAX_QUEUE_SIZE;
    q->vehicles[q->rear] = v;
    count++;
}

void dequeue(){

}