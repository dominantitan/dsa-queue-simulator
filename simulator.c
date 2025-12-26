#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define MAX_LINE_LENGTH 20
#define MAIN_FONT "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define SCALE 1
#define ROAD_WIDTH 150
#define LANE_WIDTH 50

//checkQueue constants
#define PRIORITY_THRESHOLD_HIGH 10
#define PRIORITY_THRESHOLD_LOW 5
#define TIME_PER_VEHICLE 2  // seconds per vehicle

//vehicle box dimensions
#define VEHICLE_WIDTH 20
#define VEHICLE_HEIGHT 20
#define VEHICLE_SPEED 100.0f  //pixels per second
#define VEHICLE_GAP 15        //gap between vehicles

//Turn probability (0-100, where 50 means 50% chance to turn right)
#define TURN_RIGHT_PROBABILITY 50

//position where vehicles wait (stop lines)
#define STOP_LINE_A (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - VEHICLE_HEIGHT - 5)
#define STOP_LINE_B (WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 5)
#define STOP_LINE_C (WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 5)
#define STOP_LINE_D (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - VEHICLE_WIDTH - 5)

//Lane X positions for vertical roads (A and B)
//Middle lane center X position
#define VERTICAL_LANE_CENTER_X (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH + LANE_WIDTH / 2)
//Lane A uses left side of middle lane
#define LANE_A_X (VERTICAL_LANE_CENTER_X - LANE_WIDTH / 4 - VEHICLE_WIDTH / 2)
//Lane B uses right side of middle lane
#define LANE_B_X (VERTICAL_LANE_CENTER_X + LANE_WIDTH / 4 - VEHICLE_WIDTH / 2)

//Lane Y positions for horizontal roads (C and D)
//Middle lane center Y position
#define HORIZONTAL_LANE_CENTER_Y (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH + LANE_WIDTH / 2)
//Lane C uses top side of middle lane
#define LANE_C_Y (HORIZONTAL_LANE_CENTER_Y - LANE_WIDTH / 4 - VEHICLE_HEIGHT / 2)
//Lane D uses bottom side of middle lane
#define LANE_D_Y (HORIZONTAL_LANE_CENTER_Y + LANE_WIDTH / 4 - VEHICLE_HEIGHT / 2)

//Outgoing lane positions (for vehicles exiting after turning)
//Lane A outgoing (vehicles from C turning right exit here - going up)
#define LANE_A_OUT_X (VERTICAL_LANE_CENTER_X + LANE_WIDTH / 4 - VEHICLE_WIDTH / 2)
//Lane B outgoing (vehicles from D turning right exit here - going down)
#define LANE_B_OUT_X (VERTICAL_LANE_CENTER_X - LANE_WIDTH / 4 - VEHICLE_WIDTH / 2)
//Lane C outgoing (vehicles from B turning right exit here - going right)
#define LANE_C_OUT_Y (HORIZONTAL_LANE_CENTER_Y + LANE_WIDTH / 4 - VEHICLE_HEIGHT / 2)
//Lane D outgoing (vehicles from A turning right exit here - going left)
#define LANE_D_OUT_Y (HORIZONTAL_LANE_CENTER_Y - LANE_WIDTH / 4 - VEHICLE_HEIGHT / 2)

//Intersection center for turning
#define INTERSECTION_CENTER_X (WINDOW_WIDTH / 2)
#define INTERSECTION_CENTER_Y (WINDOW_HEIGHT / 2)

//Visual-only lane positions (L1 and L3 - no queue, just visual)
//Lanes go CLOCKWISE: AL1,AL2,AL3,CL1,CL2,CL3,BL1,BL2,BL3,DL1,DL2,DL3

//Road A (top, vehicles go DOWN): AL1=leftmost, AL2=middle, AL3=rightmost
#define LANE_AL1_X (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH / 2 - VEHICLE_WIDTH / 2)  //leftmost
#define LANE_AL3_X (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * 2 + LANE_WIDTH / 2 - VEHICLE_WIDTH / 2)  //rightmost

//Road C (right, vehicles go LEFT): CL1=topmost, CL2=middle, CL3=bottommost
#define LANE_CL1_Y (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH / 2 - VEHICLE_HEIGHT / 2)  //topmost
#define LANE_CL3_Y (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * 2 + LANE_WIDTH / 2 - VEHICLE_HEIGHT / 2)  //bottommost

//Road B (bottom, vehicles go UP): BL1=rightmost, BL2=middle, BL3=leftmost
#define LANE_BL1_X (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * 2 + LANE_WIDTH / 2 - VEHICLE_WIDTH / 2)  //rightmost
#define LANE_BL3_X (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH / 2 - VEHICLE_WIDTH / 2)  //leftmost

//Road D (left, vehicles go RIGHT): DL1=bottommost, DL2=middle, DL3=topmost
#define LANE_DL1_Y (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * 2 + LANE_WIDTH / 2 - VEHICLE_HEIGHT / 2)  //bottommost
#define LANE_DL3_Y (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH / 2 - VEHICLE_HEIGHT / 2)  //topmost

//Visual vehicle spawn interval (in milliseconds)
#define VISUAL_SPAWN_INTERVAL_MIN 1000
#define VISUAL_SPAWN_INTERVAL_MAX 3000
#define MAX_VISUAL_VEHICLES 50

const char *VEHICLE_FILE = "vehicles.data";

typedef struct QueueData QueueData;

typedef struct
{
    int currentLight;
    int nextLight;
    struct QueueData *queueData;
    SDL_mutex *mutex;
} SharedData;

//Turn direction enum
typedef enum {
    TURN_STRAIGHT = 0,
    TURN_RIGHT = 1
} TurnDirection;

//Visual-only vehicle (for L1 and L3 lanes - no queue needed)
typedef struct {
    float x, y;
    bool active;
    bool hasCompletedTurn;  //true after turn complete, now moving to exit
} VisualVehicle;

//Global visual vehicle arrays
VisualVehicle visualVehiclesAL3[MAX_VISUAL_VEHICLES];  //AL3 outgoing to CL1 (turns left)
VisualVehicle visualVehiclesCL3[MAX_VISUAL_VEHICLES];  //CL3 outgoing to BL1 (turns left)
VisualVehicle visualVehiclesBL3[MAX_VISUAL_VEHICLES];  //BL3 outgoing to DL1 (turns left)
VisualVehicle visualVehiclesDL3[MAX_VISUAL_VEHICLES];  //DL3 outgoing to AL1 (turns left)

Uint32 lastSpawnTimeAL3 = 0;
Uint32 nextSpawnIntervalAL3 = 0;
Uint32 lastSpawnTimeCL3 = 0;
Uint32 nextSpawnIntervalCL3 = 0;
Uint32 lastSpawnTimeBL3 = 0;
Uint32 nextSpawnIntervalBL3 = 0;
Uint32 lastSpawnTimeDL3 = 0;
Uint32 nextSpawnIntervalDL3 = 0;

// Node for queue
typedef struct VehicleNode
{
    char vehicleNumber[10];
    char road;
    float x, y;
    float targetX, targetY;
    bool isMoving;
    bool hasCrossed;
    bool isTurning;           //true if vehicle is currently in turning phase
    bool hasCompletedTurn;    //true if turn is complete, now going straight
    TurnDirection turnDirection;
    struct VehicleNode *next;
} VehicleNode;

// Queue
typedef struct Queue
{
    VehicleNode *front;
    VehicleNode *rear;
    int size;
} Queue;

typedef struct QueueData
{
    Queue *queueA;
    Queue *queueB;
    Queue *queueC;
    Queue *queueD;
    int currentLane;  // 0 1 2 3 for A B C D
    int priorityMode; // 0 for normal and 1 fr priority
    SDL_mutex *mutex;
    int activeLane;//which lane has green light
} QueueData;

// Function declarations
bool initializeSDL(SDL_Window **window, SDL_Renderer **renderer);
void drawRoadsAndLane(SDL_Renderer *renderer, TTF_Font *font);
void displayText(SDL_Renderer *renderer, TTF_Font *font, char *text, int x, int y);
void drawTrafficLight(SDL_Renderer *renderer, int lane, bool isGreen);
void drawAllTrafficLights(SDL_Renderer *renderer, int activeLane);
void refreshLight(SDL_Renderer *renderer, SharedData *sharedData, TTF_Font *font);
void *checkQueue(void *arg);
void *readAndParseFile(void *arg);
void initQueue(Queue *queue);
void enqueue(Queue *queue, const char *vehicleNumber, char road);
VehicleNode *dequeue(Queue *queue);
int getQueueSize(Queue *queue);
void freeQueue(Queue *queue);
void drawVehicles(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData);
void drawQueueStatus(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData);
void updateVehicles(QueueData *queueData, float deltaTime);
float getStopPositionX(char road, int queuePosition);
float getStopPositionY(char road, int queuePosition);
float getSpawnPositionX(char road, Queue *queue);
float getSpawnPositionY(char road, Queue *queue);
VehicleNode *findVehicleAhead(Queue *queue, VehicleNode *current);
bool canMoveForward(VehicleNode *current, VehicleNode *ahead, char road);
bool isAnyVehicleCrossingIntersection(QueueData *queueData);
VehicleNode *findLastNonCrossedVehicle(Queue *queue);
int getWaitingVehicleCount(Queue *queue);
bool isVehicleInIntersection(VehicleNode *vehicle);
void setVehicleExitTarget(VehicleNode *vehicle);
void setVehicleTurnTarget(VehicleNode *vehicle);
void setVehicleStraightTarget(VehicleNode *vehicle);
TurnDirection getRandomTurnDirection(void);
char getRightTurnDestination(char road);

//Visual vehicle functions
void initVisualVehicles(void);
void updateVisualVehicles(float deltaTime);
void drawVisualVehicles(SDL_Renderer *renderer);


//Get random turn direction
TurnDirection getRandomTurnDirection(void)
{
    int random = rand() % 100;
    if (random < TURN_RIGHT_PROBABILITY) {
        return TURN_RIGHT;
    }
    return TURN_STRAIGHT;
}

//Get destination road for right turn
char getRightTurnDestination(char road)
{
    switch (road) {
        case 'A': return 'D';  // A turns right to D (going left)
        case 'B': return 'C';  // B turns right to C (going right)
        case 'C': return 'A';  // C turns right to A (going up)
        case 'D': return 'B';  // D turns right to B (going down)
        default: return road;
    }
}

//Set target for vehicle going straight through intersection
void setVehicleStraightTarget(VehicleNode *vehicle)
{
    switch (vehicle->road) {
        case 'A':
            vehicle->targetX = vehicle->x;
            vehicle->targetY = WINDOW_HEIGHT + VEHICLE_HEIGHT + 50;
            break;
        case 'B':
            vehicle->targetX = vehicle->x;
            vehicle->targetY = -VEHICLE_HEIGHT - 50;
            break;
        case 'C':
            vehicle->targetX = -VEHICLE_WIDTH - 50;
            vehicle->targetY = vehicle->y;
            break;
        case 'D':
            vehicle->targetX = WINDOW_WIDTH + VEHICLE_WIDTH + 50;
            vehicle->targetY = vehicle->y;
            break;
    }
}

//Set intermediate target for turning (intersection center area)
void setVehicleTurnTarget(VehicleNode *vehicle)
{
    //Move to the turning point aligned with the outgoing lane
    switch (vehicle->road) {
        case 'A':
            //A turning right: move down to D's outgoing lane Y position, then turn left
            vehicle->targetX = vehicle->x;
            vehicle->targetY = LANE_D_OUT_Y;  //turn at D's outgoing lane position
            break;
        case 'B':
            //B turning right: move up to C's outgoing lane Y position, then turn right
            vehicle->targetX = vehicle->x;
            vehicle->targetY = LANE_C_OUT_Y;  //turn at C's outgoing lane position
            break;
        case 'C':
            //C turning right: move left to A's outgoing lane X position, then turn up
            vehicle->targetX = LANE_A_OUT_X;  //turn at A's outgoing lane position
            vehicle->targetY = vehicle->y;
            break;
        case 'D':
            //D turning right: move right to B's outgoing lane X position, then turn down
            vehicle->targetX = LANE_B_OUT_X;  //turn at B's outgoing lane position
            vehicle->targetY = vehicle->y;
            break;
    }
}

//Set final exit target after completing turn
void setVehicleTurnExitTarget(VehicleNode *vehicle)
{
    switch (vehicle->road) {
        case 'A':
            //A turned right, now going left (exit on D's outgoing lane)
            vehicle->targetX = -VEHICLE_WIDTH - 50;
            vehicle->targetY = LANE_D_OUT_Y;
            break;
        case 'B':
            //B turned right, now going right (exit on C's outgoing lane)
            vehicle->targetX = WINDOW_WIDTH + VEHICLE_WIDTH + 50;
            vehicle->targetY = LANE_C_OUT_Y;
            break;
        case 'C':
            //C turned right, now going up (exit on A's outgoing lane)
            vehicle->targetX = LANE_A_OUT_X;
            vehicle->targetY = -VEHICLE_HEIGHT - 50;
            break;
        case 'D':
            //D turned right, now going down (exit on B's outgoing lane)
            vehicle->targetX = LANE_B_OUT_X;
            vehicle->targetY = WINDOW_HEIGHT + VEHICLE_HEIGHT + 50;
            break;
    }
}

//Find the last vehicle that hasn't crossed yet
VehicleNode *findLastNonCrossedVehicle(Queue *queue)
{
    VehicleNode *last = NULL;
    VehicleNode *current = queue->front;
    while (current != NULL) {
        if (!current->hasCrossed) {
            last = current;
        }
        current = current->next;
    }
    return last;
}

//Get count of vehicles waiting (not crossed yet)
int getWaitingVehicleCount(Queue *queue)
{
    int count = 0;
    VehicleNode *current = queue->front;
    while (current != NULL) {
        if (!current->hasCrossed) {
            count++;
        }
        current = current->next;
    }
    return count;
}

//Initialize visual vehicle arrays
void initVisualVehicles(void)
{
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        visualVehiclesAL3[i].active = false;
        visualVehiclesAL3[i].hasCompletedTurn = false;
        
        visualVehiclesCL3[i].active = false;
        visualVehiclesCL3[i].hasCompletedTurn = false;
        
        visualVehiclesBL3[i].active = false;
        visualVehiclesBL3[i].hasCompletedTurn = false;
        
        visualVehiclesDL3[i].active = false;
        visualVehiclesDL3[i].hasCompletedTurn = false;
    }
    Uint32 startTime = SDL_GetTicks();
    lastSpawnTimeAL3 = startTime;
    lastSpawnTimeCL3 = startTime;
    lastSpawnTimeBL3 = startTime;
    lastSpawnTimeDL3 = startTime;
    nextSpawnIntervalAL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    nextSpawnIntervalCL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    nextSpawnIntervalBL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    nextSpawnIntervalDL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
}

//Spawn visual vehicles - static helper functions
static void spawnVisualVehicleAL3(void)
{
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (!visualVehiclesAL3[i].active) {
            visualVehiclesAL3[i].active = true;
            visualVehiclesAL3[i].x = LANE_AL3_X;
            visualVehiclesAL3[i].y = -VEHICLE_HEIGHT - VEHICLE_GAP;
            visualVehiclesAL3[i].hasCompletedTurn = false;
            break;
        }
    }
}

static void spawnVisualVehicleCL3(void)
{
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (!visualVehiclesCL3[i].active) {
            visualVehiclesCL3[i].active = true;
            visualVehiclesCL3[i].x = WINDOW_WIDTH + VEHICLE_WIDTH + VEHICLE_GAP;
            visualVehiclesCL3[i].y = LANE_CL3_Y;
            visualVehiclesCL3[i].hasCompletedTurn = false;
            break;
        }
    }
}

static void spawnVisualVehicleBL3(void)
{
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (!visualVehiclesBL3[i].active) {
            visualVehiclesBL3[i].active = true;
            visualVehiclesBL3[i].x = LANE_BL3_X;
            visualVehiclesBL3[i].y = WINDOW_HEIGHT + VEHICLE_HEIGHT + VEHICLE_GAP;
            visualVehiclesBL3[i].hasCompletedTurn = false;
            break;
        }
    }
}

static void spawnVisualVehicleDL3(void)
{
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (!visualVehiclesDL3[i].active) {
            visualVehiclesDL3[i].active = true;
            visualVehiclesDL3[i].x = -VEHICLE_WIDTH - VEHICLE_GAP;
            visualVehiclesDL3[i].y = LANE_DL3_Y;
            visualVehiclesDL3[i].hasCompletedTurn = false;
            break;
        }
    }
}

//Update visual vehicles movement
void updateVisualVehicles(float deltaTime)
{
    Uint32 currentTime = SDL_GetTicks();
    
    //Spawn new vehicles at random intervals
    if (currentTime - lastSpawnTimeAL3 >= nextSpawnIntervalAL3) {
        spawnVisualVehicleAL3();
        lastSpawnTimeAL3 = currentTime;
        nextSpawnIntervalAL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    }
    if (currentTime - lastSpawnTimeCL3 >= nextSpawnIntervalCL3) {
        spawnVisualVehicleCL3();
        lastSpawnTimeCL3 = currentTime;
        nextSpawnIntervalCL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    }
    if (currentTime - lastSpawnTimeBL3 >= nextSpawnIntervalBL3) {
        spawnVisualVehicleBL3();
        lastSpawnTimeBL3 = currentTime;
        nextSpawnIntervalBL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    }
    if (currentTime - lastSpawnTimeDL3 >= nextSpawnIntervalDL3) {
        spawnVisualVehicleDL3();
        lastSpawnTimeDL3 = currentTime;
        nextSpawnIntervalDL3 = VISUAL_SPAWN_INTERVAL_MIN + rand() % (VISUAL_SPAWN_INTERVAL_MAX - VISUAL_SPAWN_INTERVAL_MIN);
    }
    
    float moveAmount = VEHICLE_SPEED * deltaTime;
    
    //Update AL3 vehicles (going down, then turn left to CL1 - exit right)
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesAL3[i].active) {
            if (!visualVehiclesAL3[i].hasCompletedTurn) {
                //Phase 1: Move down to turning point
                visualVehiclesAL3[i].y += moveAmount;
                if (visualVehiclesAL3[i].y >= LANE_CL1_Y) {
                    visualVehiclesAL3[i].y = LANE_CL1_Y;
                    visualVehiclesAL3[i].hasCompletedTurn = true;
                }
            } else {
                //Phase 2: Move right to exit
                visualVehiclesAL3[i].x += moveAmount;
            }
            //Deactivate if off-screen
            if (visualVehiclesAL3[i].x > WINDOW_WIDTH + VEHICLE_WIDTH) {
                visualVehiclesAL3[i].active = false;
            }
        }
    }
    
    //Update CL3 vehicles (going left, then turn left to BL1 - exit down)
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesCL3[i].active) {
            if (!visualVehiclesCL3[i].hasCompletedTurn) {
                //Phase 1: Move left to turning point
                visualVehiclesCL3[i].x -= moveAmount;
                if (visualVehiclesCL3[i].x <= LANE_BL1_X) {
                    visualVehiclesCL3[i].x = LANE_BL1_X;
                    visualVehiclesCL3[i].hasCompletedTurn = true;
                }
            } else {
                //Phase 2: Move down to exit
                visualVehiclesCL3[i].y += moveAmount;
            }
            //Deactivate if off-screen
            if (visualVehiclesCL3[i].y > WINDOW_HEIGHT + VEHICLE_HEIGHT) {
                visualVehiclesCL3[i].active = false;
            }
        }
    }
    
    //Update BL3 vehicles (going up, then turn left to DL1 - exit left)
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesBL3[i].active) {
            if (!visualVehiclesBL3[i].hasCompletedTurn) {
                //Phase 1: Move up to turning point
                visualVehiclesBL3[i].y -= moveAmount;
                if (visualVehiclesBL3[i].y <= LANE_DL1_Y) {
                    visualVehiclesBL3[i].y = LANE_DL1_Y;
                    visualVehiclesBL3[i].hasCompletedTurn = true;
                }
            } else {
                //Phase 2: Move left to exit
                visualVehiclesBL3[i].x -= moveAmount;
            }
            //Deactivate if off-screen
            if (visualVehiclesBL3[i].x < -VEHICLE_WIDTH) {
                visualVehiclesBL3[i].active = false;
            }
        }
    }
    
    //Update DL3 vehicles (going right, then turn left to AL1 - exit up)
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesDL3[i].active) {
            if (!visualVehiclesDL3[i].hasCompletedTurn) {
                //Phase 1: Move right to turning point
                visualVehiclesDL3[i].x += moveAmount;
                if (visualVehiclesDL3[i].x >= LANE_AL1_X) {
                    visualVehiclesDL3[i].x = LANE_AL1_X;
                    visualVehiclesDL3[i].hasCompletedTurn = true;
                }
            } else {
                //Phase 2: Move up to exit
                visualVehiclesDL3[i].y -= moveAmount;
            }
            //Deactivate if off-screen
            if (visualVehiclesDL3[i].y < -VEHICLE_HEIGHT) {
                visualVehiclesDL3[i].active = false;
            }
        }
    }
}

//Draw visual-only vehicles
void drawVisualVehicles(SDL_Renderer *renderer)
{
    //Draw AL3 vehicles - BLUE (same as Road A: 0, 100, 255)
    SDL_SetRenderDrawColor(renderer, 0, 100, 255, 255);
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesAL3[i].active) {
            SDL_Rect vehicleRect = {
                (int)visualVehiclesAL3[i].x,
                (int)visualVehiclesAL3[i].y,
                VEHICLE_WIDTH,
                VEHICLE_HEIGHT
            };
            SDL_RenderFillRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 0, 100, 255, 255);
        }
    }
    
    //Draw CL3 vehicles - GREEN (same as Road C: 50, 255, 50)
    SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255);
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesCL3[i].active) {
            SDL_Rect vehicleRect = {
                (int)visualVehiclesCL3[i].x,
                (int)visualVehiclesCL3[i].y,
                VEHICLE_WIDTH,
                VEHICLE_HEIGHT
            };
            SDL_RenderFillRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255);
        }
    }
    
    //Draw BL3 vehicles - RED (same as Road B: 255, 50, 50)
    SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesBL3[i].active) {
            SDL_Rect vehicleRect = {
                (int)visualVehiclesBL3[i].x,
                (int)visualVehiclesBL3[i].y,
                VEHICLE_WIDTH,
                VEHICLE_HEIGHT
            };
            SDL_RenderFillRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 255, 50, 50, 255);
        }
    }
    
    //Draw DL3 vehicles - YELLOW (same as Road D: 255, 255, 50)
    SDL_SetRenderDrawColor(renderer, 255, 255, 50, 255);
    for (int i = 0; i < MAX_VISUAL_VEHICLES; i++) {
        if (visualVehiclesDL3[i].active) {
            SDL_Rect vehicleRect = {
                (int)visualVehiclesDL3[i].x,
                (int)visualVehiclesDL3[i].y,
                VEHICLE_WIDTH,
                VEHICLE_HEIGHT
            };
            SDL_RenderFillRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &vehicleRect);
            SDL_SetRenderDrawColor(renderer, 255, 255, 50, 255);
        }
    }
}

//Get spawn position for new vehicle - always off-screen
float getSpawnPositionX(char road, Queue *queue)
{
    VehicleNode *lastNonCrossed = findLastNonCrossedVehicle(queue);
    
    switch (road) {
        case 'A': return LANE_A_X;
        case 'B': return LANE_B_X;
        case 'C': {
            //spawn off-screen to the right, behind last non-crossed vehicle
            float baseSpawn = WINDOW_WIDTH + VEHICLE_WIDTH + VEHICLE_GAP;
            if (lastNonCrossed != NULL) {
                float lastX = lastNonCrossed->x;
                if (lastNonCrossed->targetX > lastX) lastX = lastNonCrossed->targetX;
                float spawnX = lastX + VEHICLE_WIDTH + VEHICLE_GAP;
                if (spawnX < baseSpawn) {
                    spawnX = baseSpawn;
                }
                return spawnX;
            }
            return baseSpawn;
        }
        case 'D': {
            //spawn off-screen to the left, behind last non-crossed vehicle
            float baseSpawn = -VEHICLE_WIDTH - VEHICLE_GAP;
            if (lastNonCrossed != NULL) {
                float lastX = lastNonCrossed->x;
                if (lastNonCrossed->targetX < lastX) lastX = lastNonCrossed->targetX;
                float spawnX = lastX - VEHICLE_WIDTH - VEHICLE_GAP;
                if (spawnX > baseSpawn) {
                    spawnX = baseSpawn;
                }
                return spawnX;
            }
            return baseSpawn;
        }
        default: return 0;
    }
}

float getSpawnPositionY(char road, Queue *queue)
{
    VehicleNode *lastNonCrossed = findLastNonCrossedVehicle(queue);
    
    switch (road) {
        case 'A': {
            //spawn off-screen at top, behind last non-crossed vehicle
            float baseSpawn = -VEHICLE_HEIGHT - VEHICLE_GAP;
            if (lastNonCrossed != NULL) {
                //use the smaller Y (further from intersection)
                float lastY = lastNonCrossed->y;
                float targetY = lastNonCrossed->targetY;
                //for A, smaller Y means further back (up)
                float furthestBack = (lastY < targetY) ? lastY : targetY;
                float spawnY = furthestBack - VEHICLE_HEIGHT - VEHICLE_GAP;
                //only use baseSpawn if it's further back
                if (spawnY > baseSpawn) {
                    return baseSpawn;
                }
                return spawnY;
            }
            return baseSpawn;
        }
        case 'B': {
            //spawn off-screen at bottom, behind last non-crossed vehicle
            float baseSpawn = WINDOW_HEIGHT + VEHICLE_HEIGHT + VEHICLE_GAP;
            if (lastNonCrossed != NULL) {
                //use the larger Y (further from intersection)
                float lastY = lastNonCrossed->y;
                float targetY = lastNonCrossed->targetY;
                //for B, larger Y means further back (down)
                float furthestBack = (lastY > targetY) ? lastY : targetY;
                float spawnY = furthestBack + VEHICLE_HEIGHT + VEHICLE_GAP;
                //only use baseSpawn if it's further back
                if (spawnY < baseSpawn) {
                    return baseSpawn;
                }
                return spawnY;
            }
            return baseSpawn;
        }
        case 'C': return LANE_C_Y;
        case 'D': return LANE_D_Y;
        default: return 0;
    }
}

void initQueue(Queue *queue){
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
}

float getStopPositionX(char road, int queuePosition)
{
    switch (road) {
        case 'A': return LANE_A_X;
        case 'B': return LANE_B_X;
        case 'C': return STOP_LINE_C + queuePosition * (VEHICLE_WIDTH + VEHICLE_GAP);
        case 'D': return STOP_LINE_D - queuePosition * (VEHICLE_WIDTH + VEHICLE_GAP);
        default: return 0;
    }
}

float getStopPositionY(char road, int queuePosition)
{
    switch (road) {
        case 'A': return STOP_LINE_A - queuePosition * (VEHICLE_HEIGHT + VEHICLE_GAP);
        case 'B': return STOP_LINE_B + queuePosition * (VEHICLE_HEIGHT + VEHICLE_GAP);
        case 'C': return LANE_C_Y;
        case 'D': return LANE_D_Y;
        default: return 0;
    }
}

void enqueue(Queue *queue,const char *vehicleNumber,char road)
{
    VehicleNode *newNode = (VehicleNode *)malloc(sizeof(VehicleNode));
    if(!newNode){
        SDL_Log("failed to allocate memory for new vehicle node");
        return;
    }

    strncpy(newNode->vehicleNumber, vehicleNumber, sizeof(newNode->vehicleNumber)-1);
    newNode->vehicleNumber[sizeof(newNode->vehicleNumber)-1] = '\0';
    newNode->road = road;
    newNode->next = NULL;
    newNode->isMoving = true;
    newNode->hasCrossed = false;
    newNode->isTurning = false;
    newNode->hasCompletedTurn = false;
    
    //Randomly decide turn direction when vehicle is created
    newNode->turnDirection = getRandomTurnDirection();

    //count non-crossed vehicles for queue position
    int queuePos = 0;
    VehicleNode *temp = queue->front;
    while (temp != NULL) {
        if (!temp->hasCrossed) {
            queuePos++;
        }
        temp = temp->next;
    }
    
    //set target position (stop line based on queue position)
    newNode->targetX = getStopPositionX(road, queuePos);
    newNode->targetY = getStopPositionY(road, queuePos);

    //set spawn position - always off-screen, behind last vehicle
    newNode->x = getSpawnPositionX(road, queue);
    newNode->y = getSpawnPositionY(road, queue);

    if (queue->rear == NULL){
        queue->front = newNode;
        queue->rear = newNode;
    }else{
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
    queue->size++;
    
    const char *turnStr = (newNode->turnDirection == TURN_RIGHT) ? "RIGHT" : "STRAIGHT";
    SDL_Log("enqueue vehicle %s to road %c [%s] at (%.0f,%.0f) -> (%.0f,%.0f) queuePos=%d", 
            vehicleNumber, road, turnStr, newNode->x, newNode->y, newNode->targetX, newNode->targetY, queuePos);
}

VehicleNode *dequeue(Queue *queue){
    if (queue->front == NULL){
        return NULL;
    }
    VehicleNode *temp = queue->front;
    queue->front = queue->front->next;

    if(queue->front == NULL){
        queue->rear = NULL;
    }

    queue->size--;
    SDL_Log("dequeue vehicle %s from road %c (Queue size: %d)", temp->vehicleNumber, temp->road, queue->size);
    return temp;
}

int getQueueSize(Queue *queue){
    return queue->size;
}

void freeQueue(Queue *queue){
    VehicleNode *current = queue->front;
    while(current != NULL){
        VehicleNode *temp = current;
        current = current->next;
        free(temp);
    }
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
}

float moveTowards(float current, float target, float maxDelta)
{
    if (fabsf(target - current) <= maxDelta) {
        return target;
    }
    return current + (target > current ? maxDelta : -maxDelta);
}

void updateQueueTargets(Queue *queue)
{
    VehicleNode *current = queue->front;
    int position = 0;
    while (current != NULL) {
        if (!current->hasCrossed) {
            current->targetX = getStopPositionX(current->road, position);
            current->targetY = getStopPositionY(current->road, position);
            current->isMoving = true;
            position++;
        }
        current = current->next;
    }
}

//Find vehicle ahead in queue (not crossed yet)
VehicleNode *findVehicleAhead(Queue *queue, VehicleNode *current)
{
    VehicleNode *temp = queue->front;
    VehicleNode *ahead = NULL;
    
    while (temp != NULL && temp != current) {
        if (!temp->hasCrossed) {
            ahead = temp;
        }
        temp = temp->next;
    }
    return ahead;
}

//Check if vehicle can move (no collision with vehicle ahead)
bool canMoveForward(VehicleNode *current, VehicleNode *ahead, char road)
{
    if (ahead == NULL) return true;
    
    float distance = 0;
    switch (road) {
        case 'A':
            distance = ahead->y - current->y - VEHICLE_HEIGHT;
            break;
        case 'B':
            distance = current->y - ahead->y - VEHICLE_HEIGHT;
            break;
        case 'C':
            distance = current->x - ahead->x - VEHICLE_WIDTH;
            break;
        case 'D':
            distance = ahead->x - current->x - VEHICLE_WIDTH;
            break;
    }
    
    return distance >= VEHICLE_GAP;
}

//Check if vehicle has entered the intersection area
bool isVehicleInIntersection(VehicleNode *vehicle)
{
    switch (vehicle->road) {
        case 'A':
            return (vehicle->y >= WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - VEHICLE_HEIGHT);
        case 'B':
            return (vehicle->y <= WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2);
        case 'C':
            return (vehicle->x <= WINDOW_WIDTH / 2 + ROAD_WIDTH / 2);
        case 'D':
            return (vehicle->x >= WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - VEHICLE_WIDTH);
    }
    return false;
}

//Check if vehicle has reached turning point (aligned with outgoing lane)
bool hasReachedTurningPoint(VehicleNode *vehicle)
{
    float tolerance = 5.0f;
    switch (vehicle->road) {
        case 'A':
            //A turns at D's outgoing lane Y position
            return (vehicle->y >= LANE_D_OUT_Y - tolerance);
        case 'B':
            //B turns at C's outgoing lane Y position
            return (vehicle->y <= LANE_C_OUT_Y + tolerance);
        case 'C':
            //C turns at A's outgoing lane X position
            return (vehicle->x <= LANE_A_OUT_X + tolerance);
        case 'D':
            //D turns at B's outgoing lane X position
            return (vehicle->x >= LANE_B_OUT_X - tolerance);
    }
    return false;
}

void updateVehicles(QueueData *queueData, float deltaTime){
    float movement = VEHICLE_SPEED * deltaTime;
    Queue *queues[] = {queueData->queueA, queueData->queueB, queueData->queueC, queueData->queueD};
    int laneIndex[] = {0, 1, 2, 3};

    for (int q = 0; q < 4; q++) {
        Queue *queue = queues[q];
        VehicleNode *current = queue->front;
        VehicleNode *prev = NULL;
        
        bool isGreenLight = (queueData->activeLane == laneIndex[q]);

        while (current != NULL) {
            VehicleNode *next = current->next;

            if (current->hasCrossed) {
                //Vehicle is crossing/turning through intersection
                
                //Check if turning and reached turning point
                if (current->turnDirection == TURN_RIGHT && current->isTurning && !current->hasCompletedTurn) {
                    if (hasReachedTurningPoint(current)) {
                        //Reached turning point, set final exit target
                        current->hasCompletedTurn = true;
                        current->isTurning = false;
                        setVehicleTurnExitTarget(current);
                        SDL_Log("Vehicle %s completed turn, heading to exit", current->vehicleNumber);
                    }
                }
                
                //Move towards target
                current->x = moveTowards(current->x, current->targetX, movement);
                current->y = moveTowards(current->y, current->targetY, movement);

                //Check if vehicle is off screen
                bool offScreen = false;
                if (current->turnDirection == TURN_RIGHT && current->hasCompletedTurn) {
                    //Check exit based on turn destination
                    switch (current->road) {
                        case 'A': offScreen = (current->x < -VEHICLE_WIDTH - 10); break;  //exits left
                        case 'B': offScreen = (current->x > WINDOW_WIDTH + VEHICLE_WIDTH + 10); break;  //exits right
                        case 'C': offScreen = (current->y < -VEHICLE_HEIGHT - 10); break;  //exits top
                        case 'D': offScreen = (current->y > WINDOW_HEIGHT + VEHICLE_HEIGHT + 10); break;  //exits bottom
                    }
                } else {
                    //Going straight
                    switch (current->road) {
                        case 'A': offScreen = (current->y > WINDOW_HEIGHT + VEHICLE_HEIGHT + 10); break;
                        case 'B': offScreen = (current->y < -VEHICLE_HEIGHT - 10); break;
                        case 'C': offScreen = (current->x < -VEHICLE_WIDTH - 10); break;
                        case 'D': offScreen = (current->x > WINDOW_WIDTH + VEHICLE_WIDTH + 10); break;
                    }
                }

                if (offScreen) {
                    if (prev == NULL) {
                        VehicleNode *removed = dequeue(queue);
                        if (removed) {
                            SDL_Log("Vehicle %s exited screen from road %c", removed->vehicleNumber, removed->road);
                            free(removed);
                        }
                    } else {
                        prev->next = next;
                        if (queue->rear == current) {
                            queue->rear = prev;
                        }
                        queue->size--;
                        SDL_Log("Vehicle %s exited screen from road %c", current->vehicleNumber, current->road);
                        free(current);
                    }
                    current = next;
                    continue;
                }
            } else if (isGreenLight) {
                //Green light - move vehicle towards intersection
                VehicleNode *ahead = findVehicleAhead(queue, current);
                
                if (isVehicleInIntersection(current)) {
                    //Vehicle entered intersection
                    current->hasCrossed = true;
                    current->isMoving = true;
                    
                    if (current->turnDirection == TURN_RIGHT) {
                        //Start turning
                        current->isTurning = true;
                        setVehicleTurnTarget(current);
                        SDL_Log("Vehicle %s entered intersection from road %c - TURNING RIGHT", 
                                current->vehicleNumber, current->road);
                    } else {
                        //Go straight
                        setVehicleStraightTarget(current);
                        SDL_Log("Vehicle %s entered intersection from road %c - GOING STRAIGHT", 
                                current->vehicleNumber, current->road);
                    }
                    
                    updateQueueTargets(queue);
                } else {
                    //Vehicle not yet in intersection - move towards it
                    if (canMoveForward(current, ahead, current->road)) {
                        //Temporarily set target to move through
                        switch (current->road) {
                            case 'A':
                                current->targetX = current->x;
                                current->targetY = WINDOW_HEIGHT + VEHICLE_HEIGHT + 50;
                                break;
                            case 'B':
                                current->targetX = current->x;
                                current->targetY = -VEHICLE_HEIGHT - 50;
                                break;
                            case 'C':
                                current->targetX = -VEHICLE_WIDTH - 50;
                                current->targetY = current->y;
                                break;
                            case 'D':
                                current->targetX = WINDOW_WIDTH + VEHICLE_WIDTH + 50;
                                current->targetY = current->y;
                                break;
                        }
                        current->isMoving = true;
                        current->x = moveTowards(current->x, current->targetX, movement);
                        current->y = moveTowards(current->y, current->targetY, movement);
                    }
                }
            } else if (!current->hasCrossed) {
                //Red light and vehicle hasn't crossed yet
                int position = 0;
                VehicleNode *temp = queue->front;
                while (temp != NULL && temp != current) {
                    if (!temp->hasCrossed) {
                        position++;
                    }
                    temp = temp->next;
                }
                current->targetX = getStopPositionX(current->road, position);
                current->targetY = getStopPositionY(current->road, position);
                
                VehicleNode *ahead = findVehicleAhead(queue, current);
                
                if (canMoveForward(current, ahead, current->road)) {
                    current->x = moveTowards(current->x, current->targetX, movement);
                    current->y = moveTowards(current->y, current->targetY, movement);
                }

                if (fabsf(current->x - current->targetX) < 0.5f && 
                    fabsf(current->y - current->targetY) < 0.5f) {
                    current->isMoving = false;
                } else {
                    current->isMoving = true;
                }
            }

            prev = current;
            current = next;
        }
    }
}

void drawVehicles(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData)
{
    Queue *queues[] = {queueData->queueA, queueData->queueB, queueData->queueC, queueData->queueD};
    SDL_Color colors[] = {
        {0, 100, 255, 255},    // Blue for A
        {255, 50, 50, 255},    // Red for B
        {50, 255, 50, 255},    // Green for C
        {255, 255, 50, 255}    // Yellow for D
    };

    for (int q = 0; q < 4; q++) {
        VehicleNode *current = queues[q]->front;
        
        while (current != NULL) {
            //Set color based on road (darker if turning)
            if (current->turnDirection == TURN_RIGHT) {
                //Darker color for turning vehicles
                SDL_SetRenderDrawColor(renderer, 
                    colors[q].r * 0.7, 
                    colors[q].g * 0.7, 
                    colors[q].b * 0.7, 
                    colors[q].a);
            } else {
                SDL_SetRenderDrawColor(renderer, colors[q].r, colors[q].g, colors[q].b, colors[q].a);
            }
            
            SDL_Rect vehicleRect = {
                (int)current->x,
                (int)current->y,
                VEHICLE_WIDTH,
                VEHICLE_HEIGHT
            };
            
            SDL_RenderFillRect(renderer, &vehicleRect);
            
            //Draw border (orange for turning, white for straight)
            if (current->turnDirection == TURN_RIGHT) {
                SDL_SetRenderDrawColor(renderer, 255, 165, 0, 255);  //Orange border for turning
            } else {
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);  //White border for straight
            }
            SDL_RenderDrawRect(renderer, &vehicleRect);
            
            current = current->next;
        }
    }
}

void drawQueueStatus(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData)
{
    char statusText[100];
    
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    SDL_Rect statusBox = {10, 10, 200, 120};
    SDL_RenderFillRect(renderer, &statusBox);
    
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderDrawRect(renderer, &statusBox);

    snprintf(statusText, sizeof(statusText), "AL2: %d vehicles", getQueueSize(queueData->queueA));
    displayText(renderer, font, statusText, 20, 20);
    
    snprintf(statusText, sizeof(statusText), "BL2: %d vehicles", getQueueSize(queueData->queueB));
    displayText(renderer, font, statusText, 20, 45);
    
    snprintf(statusText, sizeof(statusText), "CL2: %d vehicles", getQueueSize(queueData->queueC));
    displayText(renderer, font, statusText, 20, 70);
    
    snprintf(statusText, sizeof(statusText), "DL2: %d vehicles", getQueueSize(queueData->queueD));
    displayText(renderer, font, statusText, 20, 95);

    if (queueData->priorityMode) {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_Rect modeBox = {WINDOW_WIDTH - 210, 10, 210, 30};
        SDL_RenderFillRect(renderer, &modeBox);
        displayText(renderer, font, "PRIORITY MODE", WINDOW_WIDTH - 195, 11);
    } else {
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        SDL_Rect modeBox = {WINDOW_WIDTH - 200, 10, 195, 30};
        SDL_RenderFillRect(renderer, &modeBox);
        displayText(renderer, font, "NORMAL MODE", WINDOW_WIDTH - 195, 11);
    }
}

int main()
{
    //Initialize random seed
    srand(time(NULL));
    
    pthread_t tQueue, tReadFile;
    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    SDL_Event event;

    if (!initializeSDL(&window, &renderer))
    {
        return -1;
    }
    SDL_mutex *mutex = SDL_CreateMutex();

    QueueData queueData;
    queueData.queueA = (Queue *)malloc(sizeof(Queue));
    queueData.queueB = (Queue *)malloc(sizeof(Queue));
    queueData.queueC = (Queue *)malloc(sizeof(Queue));
    queueData.queueD = (Queue *)malloc(sizeof(Queue));

    initQueue(queueData.queueA);
    initQueue(queueData.queueB);
    initQueue(queueData.queueC);
    initQueue(queueData.queueD);
    
    //Initialize visual-only vehicles (L1 and L3 lanes)
    initVisualVehicles();

    queueData.currentLane = 0;
    queueData.priorityMode = 0;
    queueData.activeLane = -1;
    queueData.mutex = mutex;

    SharedData sharedData = {0, 0, &queueData, mutex};

    TTF_Font *font = TTF_OpenFont(MAIN_FONT, 24);
    if (!font){
        SDL_Log("Failed to load font: %s", TTF_GetError());
    }
    
    pthread_create(&tQueue, NULL, checkQueue, &sharedData);
    pthread_create(&tReadFile, NULL, readAndParseFile, &queueData);

    const int TARGET_FPS = 60;
    const int FRAME_DELAY = 1000 / TARGET_FPS;  // ~16ms per frame
    Uint32 frameStart;
    Uint32 frameTime;
    Uint32 lastTime = SDL_GetTicks();
    float deltaTime;

    bool running = true;
    while (running)
    {
        frameStart = SDL_GetTicks();
        deltaTime = (frameStart - lastTime) / 1000.0f;
        
        // Cap deltaTime to prevent huge jumps
        if (deltaTime > 0.1f) deltaTime = 0.1f;
        lastTime = frameStart;
        
        while (SDL_PollEvent(&event)){
            if (event.type == SDL_QUIT)
                running = false;
        }
        
        SDL_LockMutex(mutex);
        updateVehicles(&queueData, deltaTime);
        updateVisualVehicles(deltaTime);
        refreshLight(renderer, &sharedData, font);
        drawVehicles(renderer, font, &queueData);
        drawVisualVehicles(renderer);
        drawQueueStatus(renderer, font, &queueData);
        SDL_UnlockMutex(mutex);

        SDL_RenderPresent(renderer);
        
        // Frame rate limiting - only delay if we finished early
        frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < FRAME_DELAY) {
            SDL_Delay(FRAME_DELAY - frameTime);
        }
    }

    SDL_DestroyMutex(mutex);
    freeQueue(queueData.queueA);
    freeQueue(queueData.queueB);
    freeQueue(queueData.queueC);
    freeQueue(queueData.queueD);
    free(queueData.queueA);
    free(queueData.queueB);
    free(queueData.queueC);
    free(queueData.queueD);
    TTF_CloseFont(font);
    TTF_Quit();
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (window)
        SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

bool initializeSDL(SDL_Window **window, SDL_Renderer **renderer)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        SDL_Log("Failed to initialize SDL: %s", SDL_GetError());
        return false;
    }
    if (TTF_Init() < 0)
    {
        SDL_Log("SDL_ttf could not initialize! TTF_Error: %s\n", TTF_GetError());
        return false;
    }

    *window = SDL_CreateWindow("Junction Diagram",
                               SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               WINDOW_WIDTH * SCALE, WINDOW_HEIGHT * SCALE,
                               SDL_WINDOW_SHOWN);
    if (!*window)
    {
        SDL_Log("Failed to create window: %s", SDL_GetError());
        SDL_Quit();
        return false;
    }

    *renderer = SDL_CreateRenderer(*window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetScale(*renderer, SCALE, SCALE);

    if (!*renderer)
    {
        SDL_Log("Failed to create renderer: %s", SDL_GetError());
        SDL_DestroyWindow(*window);
        TTF_Quit();
        SDL_Quit();
        return false;
    }

    return true;
}

void drawRoadsAndLane(SDL_Renderer *renderer, TTF_Font *font)
{
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    
    SDL_Rect verticalRoad = {WINDOW_WIDTH / 2 - ROAD_WIDTH / 2, 0, ROAD_WIDTH, WINDOW_HEIGHT};
    SDL_RenderFillRect(renderer, &verticalRoad);

    SDL_Rect horizontalRoad = {0, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2, WINDOW_WIDTH, ROAD_WIDTH};
    SDL_RenderFillRect(renderer, &horizontalRoad);
    
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    for (int i = 0; i <= 3; i++)
    {
        SDL_RenderDrawLine(renderer,
                           0, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i,
                           WINDOW_WIDTH / 2 - ROAD_WIDTH / 2, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i
        );
        SDL_RenderDrawLine(renderer,
                           800, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i,
                           WINDOW_WIDTH / 2 + ROAD_WIDTH / 2, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i);
        SDL_RenderDrawLine(renderer,
                           WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i, 0,
                           WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2);
        SDL_RenderDrawLine(renderer,
                           WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i, 800,
                           WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i, WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2);
    }
    displayText(renderer, font, "A", 400, 10);
    displayText(renderer, font, "B", 400, 770);
    displayText(renderer, font, "D", 10, 400);
    displayText(renderer, font, "C", 770, 400);
}

void drawTrafficLight(SDL_Renderer *renderer, int lane, bool isGreen)
{
    int x, y;
    switch (lane) {
        case 0:
            x = WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 10;
            y = WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 40;
            break;
        case 1:
            x = WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 50;
            y = WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 10;
            break;
        case 2:
            x = WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 10;
            y = WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 10;
            break;
        case 3:
            x = WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 50;
            y = WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 40;
            break;
        default:
            return;
    }

    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_Rect lightBox = {x, y, 40, 30};
    SDL_RenderFillRect(renderer, &lightBox);

    if (!isGreen)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 100, 0, 0, 255);
    SDL_Rect redLight = {x + 5, y + 5, 12, 12};
    SDL_RenderFillRect(renderer, &redLight);

    if (isGreen)
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 0, 100, 0, 255);
    SDL_Rect greenLight = {x + 22, y + 5, 12, 12};
    SDL_RenderFillRect(renderer, &greenLight);
}

void drawAllTrafficLights(SDL_Renderer *renderer, int activeLane)
{
    for (int i = 0; i < 4; i++) {
        drawTrafficLight(renderer, i, (i == activeLane));
    }
}

void displayText(SDL_Renderer *renderer, TTF_Font *font, char *text, int x, int y)
{
    SDL_Color textColor = {255, 255, 255, 255};
    SDL_Surface *textSurface = TTF_RenderText_Solid(font, text, textColor);
    if (!textSurface) return;
    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_FreeSurface(textSurface);
    if (!texture) return;
    SDL_Rect textRect = {x, y, 0, 0};
    SDL_QueryTexture(texture, NULL, NULL, &textRect.w, &textRect.h);
    SDL_RenderCopy(renderer, texture, NULL, &textRect);
    SDL_DestroyTexture(texture);
}

void refreshLight(SDL_Renderer *renderer, SharedData *sharedData, TTF_Font *font)
{
    SDL_SetRenderDrawColor(renderer, 34, 139, 34, 255);
    SDL_RenderClear(renderer);

    drawRoadsAndLane(renderer, font);

    int activeLane = sharedData->nextLight - 1;
    drawAllTrafficLights(renderer, activeLane);
}

//Check if any vehicle is still crossing the intersection (any lane)
bool isAnyVehicleCrossingIntersection(QueueData *queueData)
{
    Queue *queues[] = {queueData->queueA, queueData->queueB, queueData->queueC, queueData->queueD};
    
    for (int q = 0; q < 4; q++) {
        VehicleNode *current = queues[q]->front;
        while (current != NULL) {
            if (current->hasCrossed) {
                bool inIntersection = false;
                
                //Check based on turn direction
                if (current->turnDirection == TURN_RIGHT && !current->hasCompletedTurn) {
                    //Still turning - definitely in intersection
                    inIntersection = true;
                } else if (current->turnDirection == TURN_RIGHT && current->hasCompletedTurn) {
                    //Completed turn - check based on exit direction
                    switch (current->road) {
                        case 'A':  //exiting left
                            inIntersection = (current->x >= WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - VEHICLE_WIDTH);
                            break;
                        case 'B':  //exiting right
                            inIntersection = (current->x <= WINDOW_WIDTH / 2 + ROAD_WIDTH / 2);
                            break;
                        case 'C':  //exiting up
                            inIntersection = (current->y >= WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - VEHICLE_HEIGHT);
                            break;
                        case 'D':  //exiting down
                            inIntersection = (current->y <= WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2);
                            break;
                    }
                } else {
                    //Going straight
                    switch (current->road) {
                        case 'A':
                            inIntersection = (current->y >= WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - VEHICLE_HEIGHT &&
                                             current->y <= WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2);
                            break;
                        case 'B':
                            inIntersection = (current->y >= WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 &&
                                             current->y <= WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + VEHICLE_HEIGHT);
                            break;
                        case 'C':
                            inIntersection = (current->x >= WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 &&
                                             current->x <= WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + VEHICLE_WIDTH);
                            break;
                        case 'D':
                            inIntersection = (current->x >= WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - VEHICLE_WIDTH &&
                                             current->x <= WINDOW_WIDTH / 2 + ROAD_WIDTH / 2);
                            break;
                    }
                }
                
                if (inIntersection) {
                    return true;
                }
            }
            current = current->next;
        }
    }
    return false;
}

void *checkQueue(void *arg)
{
    SharedData *sharedData = (SharedData *)arg;
    QueueData *queueData = sharedData->queueData;
    while (1)
    {
        while (1) {
            SDL_LockMutex(queueData->mutex);
            bool intersectionBusy = isAnyVehicleCrossingIntersection(queueData);
            SDL_UnlockMutex(queueData->mutex);
            
            if (!intersectionBusy) {
                break;
            }
            SDL_Delay(50);
        }
        
        SDL_LockMutex(queueData->mutex);

        int sizeA = getWaitingVehicleCount(queueData->queueA);
        int sizeB = getWaitingVehicleCount(queueData->queueB);
        int sizeC = getWaitingVehicleCount(queueData->queueC);
        int sizeD = getWaitingVehicleCount(queueData->queueD);

        //Priority mode activation: > 10 vehicles triggers priority mode
        //Priority mode deactivation: < 5 vehicles exits priority mode
        if (sizeA > PRIORITY_THRESHOLD_HIGH){
            queueData->priorityMode = 1;
            SDL_Log("Priority mode activated!! lane A has %d vehicles", sizeA);
        }else if(sizeA < PRIORITY_THRESHOLD_LOW && queueData->priorityMode == 1){
            queueData->priorityMode = 0;
            SDL_Log("Normal Mode continued!! lane A has %d vehicle", sizeA);
        }

        int laneToServe;
        int vehiclesToServe = 0;

        if(queueData->priorityMode == 1){
            //Full priority mode: serve only lane A until < 5 vehicles
            laneToServe = 0;
            vehiclesToServe = sizeA;
            SDL_Log("Priority mode: serving lane A with %d vehicles", vehiclesToServe);
        } else if (sizeA > PRIORITY_THRESHOLD_LOW && queueData->currentLane != 0) {
            //Immediate service: lane A has > 5 vehicles, serve it next (but not full priority)
            laneToServe = 0;
            
            //Calculate average from all 4 lanes for fair serving
            int totalVehicles = sizeA + sizeB + sizeC + sizeD;
            int avgVehicles = (totalVehicles + 3) / 4;  //round up division by 4
            if (avgVehicles < 1) avgVehicles = 1;
            
            vehiclesToServe = (sizeA < avgVehicles) ? sizeA : avgVehicles;
            if (vehiclesToServe < 1 && sizeA > 0) vehiclesToServe = 1;
            
            SDL_Log("Immediate service for lane A (>5 vehicles): size=%d, avg=%d, serving %d vehicles", 
                    sizeA, avgVehicles, vehiclesToServe);
            //Don't change currentLane - will continue normal rotation after this
        } else {
            //Normal mode: serve lanes equally in rotation
            //Calculate average from all 4 lanes
            int totalVehicles = sizeA + sizeB + sizeC + sizeD;
            int avgVehicles = (totalVehicles + 3) / 4;  //round up division by 4
            
            if (avgVehicles < 1) avgVehicles = 1;
            
            laneToServe = queueData->currentLane;
            
            int currentLaneSize = 0;
            switch (laneToServe) {
                case 0: currentLaneSize = sizeA; break;
                case 1: currentLaneSize = sizeB; break;
                case 2: currentLaneSize = sizeC; break;
                case 3: currentLaneSize = sizeD; break;
            }
            
            vehiclesToServe = (currentLaneSize < avgVehicles) ? currentLaneSize : avgVehicles;
            if (vehiclesToServe < 1 && currentLaneSize > 0) vehiclesToServe = 1;
            
            SDL_Log("Normal mode: lane %d, size=%d, avg=%d, serving %d vehicles", 
                    laneToServe, currentLaneSize, avgVehicles, vehiclesToServe);
            
            queueData->currentLane = (queueData->currentLane + 1) % 4;
        }

        SDL_UnlockMutex(queueData->mutex);

        if (vehiclesToServe > 0) {
            sharedData->nextLight = laneToServe + 1;
            queueData->activeLane = laneToServe;
            
            int greenLightTime = vehiclesToServe * TIME_PER_VEHICLE * 1000;
            SDL_Log("Green light for lane %d for %d ms (%d vehicles * %d sec)", 
                    laneToServe, greenLightTime, vehiclesToServe, TIME_PER_VEHICLE);
            
            SDL_Delay(greenLightTime);
            
            sharedData->nextLight = 0;
            queueData->activeLane = -1;
            SDL_Log("Red light for lane %d - waiting for crossing vehicles to clear", laneToServe);
            
        } else {
            SDL_Log("No vehicles in lane %d, skipping", laneToServe);
            SDL_Delay(200);
        }
    }
    return NULL;
}

void *readAndParseFile(void *arg)
{
    QueueData *queueData = (QueueData *)arg;
    long lastFilePos = 0;
    while (1)
    {
        FILE *file = fopen(VEHICLE_FILE, "r");
        if (!file)
        {
            SDL_Log("waiting for vehicle file '%s'...", VEHICLE_FILE);
            sleep(2);
            continue;
        }

        fseek(file, lastFilePos, SEEK_SET);

        char line[MAX_LINE_LENGTH];
        while (fgets(line, sizeof(line), file))
        {
            line[strcspn(line, "\n")] = 0;
            
            if (strlen(line) == 0) continue;

            char *vehicleNumber = strtok(line, ":");
            char *roadStr = strtok(NULL, ":");

            if (vehicleNumber && roadStr){
                char road = roadStr[0];
                SDL_LockMutex(queueData->mutex);

                switch (road){
                    case 'A':
                        enqueue(queueData->queueA, vehicleNumber, road);
                        break;
                    case 'B':
                        enqueue(queueData->queueB, vehicleNumber, road);
                        break;
                    case 'C':
                        enqueue(queueData->queueC, vehicleNumber, road);
                        break;
                    case 'D':
                        enqueue(queueData->queueD, vehicleNumber, road);
                        break;
                    default:
                        SDL_Log("Unknown road: %c", road);
                }
                
                SDL_UnlockMutex(queueData->mutex);
            }
        }
        lastFilePos = ftell(file);
        fclose(file);
        sleep(1);
    }
    return NULL;
}