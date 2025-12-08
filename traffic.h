#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MAX_QUEUE_SIZE 100
#define PRIORITY_ADD 10  // If AL2 > 10, it gets priority
#define PRIORITY_REMOVE 5       // If AL2 < 5, priority drops
#define GREENLIGHT_DURATION 5 //duration of red light

//giving priority value to the lanes
typedef enum {
    AL2 = 0, // Road A, Lane 2 (The Priority Lane)
    BL2 = 1, // Road B, Lane 2
    CL2 = 2, // Road C, Lane 2
    DL2 = 3  // Road D, Lane 2
} LaneID;

//vehicle structure
typedef struct {
    int id;           // Unique ID for the car
    time_t arrivalTime; // When this car arrive?
    LaneID lane;       // 0=A, 1=B, 2=C, 3=D
} Vehicle;

//queue for the vehicles
typedef struct {
    Vehicle vehicles[MAX_QUEUE_SIZE];
    int front;
    int rear;
    int count; // Current number of vehicles
} VehicleQueue;

typedef struct {
    LaneID currentGreenLane; //which lane has green light?
    int timer;  //how long has the green light been on?
    int isPriorityMode; //1 for emergency AL2 0 for normal 
}TrafficSystem;
