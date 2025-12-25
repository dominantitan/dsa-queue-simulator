#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
#define VEHICLE_GAP 10        //gap between vehicles

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

const char *VEHICLE_FILE = "vehicles.data";

typedef struct QueueData QueueData;

typedef struct
{
    int currentLight;
    int nextLight;
    struct QueueData *queueData;
    SDL_mutex *mutex;
} SharedData;

// Node for queue
typedef struct VehicleNode
{
    char vehicleNumber[10];// unique id for the vehicle
    char road;//road the vehicle is in 
    float x,y;
    float targetX,targetY;
    bool isMoving;
    bool hasCrossed;
    struct VehicleNode *next; // pointer to point at next vehiclenode in queue
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

    strncpy(newNode->vehicleNumber,vehicleNumber,sizeof(newNode->vehicleNumber)-1);
    newNode->vehicleNumber[sizeof(newNode->vehicleNumber)-1] = '\0';
    newNode->road = road;
    newNode->next = NULL;
    newNode->isMoving = true;
    newNode->hasCrossed = false;

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
    SDL_Log("enqueue vehicle %s to road %c at (%.0f,%.0f) -> (%.0f,%.0f) queuePos=%d", 
            vehicleNumber, road, newNode->x, newNode->y, newNode->targetX, newNode->targetY, queuePos);
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
        VehicleNode *temp =current;
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
    //Update target positions for all vehicles in queue after one crosses
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
            //current is above ahead (smaller Y), ahead is closer to intersection (larger Y)
            distance = ahead->y - current->y - VEHICLE_HEIGHT;
            break;
        case 'B':
            //current is below ahead (larger Y), ahead is closer to intersection (smaller Y)
            distance = current->y - ahead->y - VEHICLE_HEIGHT;
            break;
        case 'C':
            //current is right of ahead (larger X), ahead is closer to intersection (smaller X)
            distance = current->x - ahead->x - VEHICLE_WIDTH;
            break;
        case 'D':
            //current is left of ahead (smaller X), ahead is closer to intersection (larger X)
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

void updateVehicles(QueueData *queueData, float deltaTime){
    float movement = VEHICLE_SPEED * deltaTime;
    Queue *queues[] = {queueData->queueA,queueData->queueB, queueData->queueC,queueData->queueD};
    int laneIndex[] = {0, 1, 2, 3};

    for (int q = 0; q < 4; q++) {
        Queue *queue = queues[q];
        VehicleNode *current = queue->front;
        VehicleNode *prev = NULL;
        
        //check if this lane has green light
        bool isGreenLight = (queueData->activeLane == laneIndex[q]);

        while (current != NULL) {
            VehicleNode *next = current->next;

            if (current->hasCrossed) {
                //Vehicle is crossing - move straight through intersection
                current->x = moveTowards(current->x, current->targetX, movement);
                current->y = moveTowards(current->y, current->targetY, movement);

                //Check if vehicle is off screen
                bool offScreen = false;
                switch (current->road) {
                    case 'A': offScreen = (current->y > WINDOW_HEIGHT + VEHICLE_HEIGHT + 10); break;
                    case 'B': offScreen = (current->y < -VEHICLE_HEIGHT - 10); break;
                    case 'C': offScreen = (current->x < -VEHICLE_WIDTH - 10); break;
                    case 'D': offScreen = (current->x > WINDOW_WIDTH + VEHICLE_WIDTH + 10); break;
                }

                if (offScreen) {
                    //Remove vehicle from queue using dequeue if at front
                    if (prev == NULL) {
                        //Vehicle is at front - use dequeue operation
                        VehicleNode *removed = dequeue(queue);
                        if (removed) {
                            SDL_Log("Vehicle %s exited screen from road %c", removed->vehicleNumber, removed->road);
                            free(removed);
                        }
                    } else {
                        //Vehicle is not at front - remove inline
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
                
                //Check if vehicle has entered the intersection
                if (isVehicleInIntersection(current)) {
                    //Vehicle entered intersection - mark as crossed and set exit target
                    current->hasCrossed = true;
                    current->isMoving = true;
                    
                    //Set exit target - go straight through to opposite side
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
                    SDL_Log("Vehicle %s entered intersection from road %c", current->vehicleNumber, current->road);
                    
                    //Update targets for remaining vehicles
                    updateQueueTargets(queue);
                } else {
                    //Vehicle not yet in intersection - move towards it with collision check
                    if (canMoveForward(current, ahead, current->road)) {
                        //Set target to intersection entry point
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
                //Reset target to stop position if not already there
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
                
                //Move towards stop line with collision detection
                VehicleNode *ahead = findVehicleAhead(queue, current);
                
                if (canMoveForward(current, ahead, current->road)) {
                    current->x = moveTowards(current->x, current->targetX, movement);
                    current->y = moveTowards(current->y, current->targetY, movement);
                }

                //Check if reached target
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
            //Set color based on road
            SDL_SetRenderDrawColor(renderer, colors[q].r, colors[q].g, colors[q].b, colors[q].a);
            
            SDL_Rect vehicleRect = {
                (int)current->x,
                (int)current->y,
                VEHICLE_WIDTH,
                VEHICLE_HEIGHT
            };
            
            SDL_RenderFillRect(renderer, &vehicleRect);
            //Draw border
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawRect(renderer, &vehicleRect);
            current = current->next;
        }
    }
}

void drawQueueStatus(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData)
{
    char statusText[100];
    
    //draw status box
    SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
    SDL_Rect statusBox = {10, 10, 200, 120};
    SDL_RenderFillRect(renderer, &statusBox);
    
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderDrawRect(renderer, &statusBox);

    //display queue sizes
    snprintf(statusText, sizeof(statusText), "A: %d vehicles", getQueueSize(queueData->queueA));
    displayText(renderer, font, statusText, 20, 20);
    
    snprintf(statusText, sizeof(statusText), "B: %d vehicles", getQueueSize(queueData->queueB));
    displayText(renderer, font, statusText, 20, 45);
    
    snprintf(statusText, sizeof(statusText), "C: %d vehicles", getQueueSize(queueData->queueC));
    displayText(renderer, font, statusText, 20, 70);
    
    snprintf(statusText, sizeof(statusText), "D: %d vehicles", getQueueSize(queueData->queueD));
    displayText(renderer, font, statusText, 20, 95);

    //display mode
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
    pthread_t tQueue, tReadFile;
    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;
    SDL_Event event;

    if (!initializeSDL(&window, &renderer))
    {
        return -1;
    }
    SDL_mutex *mutex = SDL_CreateMutex();

    //initializing queue dat 
    QueueData queueData;
    queueData.queueA = (Queue *)malloc(sizeof(Queue));
    queueData.queueB = (Queue *)malloc(sizeof(Queue));
    queueData.queueC = (Queue *)malloc(sizeof(Queue));
    queueData.queueD = (Queue *)malloc(sizeof(Queue));

    initQueue(queueData.queueA);
    initQueue(queueData.queueB);
    initQueue(queueData.queueC);
    initQueue(queueData.queueD);

    queueData.currentLane = 0;// start with lane A
    queueData.priorityMode = 0;// normal mode
    queueData.activeLane = -1;
    queueData.mutex = mutex;

    SharedData sharedData = {0, 0, &queueData, mutex};
    // 0 => all red

    TTF_Font *font = TTF_OpenFont(MAIN_FONT, 24);
    if (!font){
        SDL_Log("Failed to load font: %s", TTF_GetError());
    }
    
    pthread_create(&tQueue,NULL,checkQueue,&sharedData);
    pthread_create(&tReadFile,NULL,readAndParseFile,&queueData);

    //Delta time variables
    Uint32 lastTime = SDL_GetTicks();
    Uint32 currentTime;
    float deltaTime;

    // Continue the UI thread
    bool running = true;
    while (running)
    {
        currentTime = SDL_GetTicks();
        deltaTime = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;
        
        while (SDL_PollEvent(&event)){
            if (event.type == SDL_QUIT)
                running = false;
        }
        
        SDL_LockMutex(mutex);
        updateVehicles(&queueData,deltaTime);
        refreshLight(renderer, &sharedData, font);
        drawVehicles(renderer, font, &queueData);
        drawQueueStatus(renderer, font, &queueData);
        SDL_UnlockMutex(mutex);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    //memory management
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
                //check if vehicle is still in the intersection area
                bool inIntersection = false;
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
        //First, wait for intersection to be clear before proceeding
        while (1) {
            SDL_LockMutex(queueData->mutex);
            bool intersectionBusy = isAnyVehicleCrossingIntersection(queueData);
            SDL_UnlockMutex(queueData->mutex);
            
            if (!intersectionBusy) {
                break;
            }
            SDL_Delay(50);  //check frequently
        }
        
        SDL_LockMutex(queueData->mutex);

        //Use waiting count instead of total queue size
        int sizeA = getWaitingVehicleCount(queueData->queueA);
        int sizeB = getWaitingVehicleCount(queueData->queueB);
        int sizeC = getWaitingVehicleCount(queueData->queueC);
        int sizeD = getWaitingVehicleCount(queueData->queueD);

        //check priority mode for lane A
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
            //Priority mode - serve lane A
            laneToServe = 0;
            vehiclesToServe = sizeA;
            SDL_Log("Priority mode: serving lane A with %d vehicles", vehiclesToServe);
        }else {
            //Normal mode - calculate average of normal lanes (B, C, D)
            //Formula: |V| = (1/n) * sum(|Li|) where n = 3 (lanes B, C, D)
            int totalNormalVehicles = sizeB + sizeC + sizeD;
            int avgVehicles = (totalNormalVehicles + 2) / 3;  // ceiling division for 3 lanes
            
            if (avgVehicles < 1) avgVehicles = 1;  //minimum 1 vehicle
            
            //rotate through lanes
            laneToServe = queueData->currentLane;
            
            //get current lane size
            int currentLaneSize = 0;
            switch (laneToServe) {
                case 0: currentLaneSize = sizeA; break;
                case 1: currentLaneSize = sizeB; break;
                case 2: currentLaneSize = sizeC; break;
                case 3: currentLaneSize = sizeD; break;
            }
            
            //serve minimum of average or current lane size
            vehiclesToServe = (currentLaneSize < avgVehicles) ? currentLaneSize : avgVehicles;
            if (vehiclesToServe < 1 && currentLaneSize > 0) vehiclesToServe = 1;
            
            SDL_Log("Normal mode: lane %d, size=%d, avg=%d, serving %d vehicles", 
                    laneToServe, currentLaneSize, avgVehicles, vehiclesToServe);
            
            //move to next lane for next cycle
            queueData->currentLane = (queueData->currentLane + 1) % 4;
        }

        SDL_UnlockMutex(queueData->mutex);

        //Check if there are vehicles to serve
        if (vehiclesToServe > 0) {
            //GREEN phase - turn on green light immediately
            sharedData->nextLight = laneToServe + 1;
            queueData->activeLane = laneToServe;
            
            //Calculate green light time: T = |V| * t
            int greenLightTime = vehiclesToServe * TIME_PER_VEHICLE * 1000;
            SDL_Log("Green light for lane %d for %d ms (%d vehicles * %d sec)", 
                    laneToServe, greenLightTime, vehiclesToServe, TIME_PER_VEHICLE);
            
            //Wait for green light duration
            SDL_Delay(greenLightTime);
            
            //Turn red - vehicles already crossing will continue
            sharedData->nextLight = 0;
            queueData->activeLane = -1;
            SDL_Log("Red light for lane %d - waiting for crossing vehicles to clear", laneToServe);
            
        } else {
            //No vehicles to serve, short delay before checking next lane
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
            SDL_Log("waiting for vehicle file '%s'...",VEHICLE_FILE);
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