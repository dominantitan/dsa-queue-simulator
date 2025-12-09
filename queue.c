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

//To add vehicles to the Queue
void enqueue(VehicleQueue *q,Vehicle v){
    if(isFull()){
        return;
    }
    q->(rear+1) % MAX_QUEUE_SIZE;
    q->vehicles[q->rear] = v;
    count++;
}

//To remove the vehicles from the queue
void dequeue(VehicleQueue *q){
    Vehicle v = {-1,0,AL2};//default empty vehicle 
    if(isEmpty()){
        return v;
    }
    v = q->vehicles[front];
    q->(front + 1) % MAX_QUEUE_SIZE;
    q->count--;
    return v;
}