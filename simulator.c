#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_LINE_LENGTH 20
#define MAIN_FONT "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define SCALE 1
#define ROAD_WIDTH 150
#define LANE_WIDTH 50
#define ARROW_SIZE 15

//checkQueue constants
#define PRIORITY_THRESHOLD_HIGH 10
#define PRIORITY_THRESHOLD_LOW 5
#define TIME_PER_VEHICLE 2  // seconds per vehicle

//vehicle box dimensions
#define VEHICLE_WIDTH 20
#define VEHICLE_HEIGHT 20
#define VEHICLE_SPEED 100.0f  //pixels per second
#define VEHICLE_GAP 10        //gap between vehicles

//position where vehicles wait
#define STOP_LINE_A (WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - VEHICLE_HEIGHT - 5)
#define STOP_LINE_B (WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 5)
#define STOP_LINE_C (WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 5)
#define STOP_LINE_D (WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - VEHICLE_WIDTH - 5)


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
void drawLightForB(SDL_Renderer *renderer, bool isRed);
void refreshLight(SDL_Renderer *renderer, SharedData *sharedData, TTF_Font *font);
void *checkQueue(void *arg);
void *readAndParseFile(void *arg);
void initQueue(Queue *queue);
void enqueue(Queue *queue, const char *vehicleNumber, char road);
VehicleNode *dequeue(Queue *queue);
int getQueueSize(Queue *queue);
VehicleNode *peekQueue(Queue *queue);
void freeQueue(Queue *queue);
void drawVehicles(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData);
void drawQueueStatus(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData);
void updateVehicles(QueueData *queueData, float deltaTime);
void removeOffScreenVehicles(QueueData *queueData);
float getInitialX(char road);
float getInitialY(char road);
float getStopPosition(char road, int queuePosition);


void initQueue(Queue *queue){
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
}

float getInitialX(char road)
{
    switch (road) {
        case 'A': return WINDOW_WIDTH / 2 - LANE_WIDTH / 2 + 15;  // Coming from top
        case 'B': return WINDOW_WIDTH / 2 + 15;                    // Coming from bottom
        case 'C': return WINDOW_WIDTH + VEHICLE_WIDTH;             // Coming from right (off-screen)
        case 'D': return -VEHICLE_WIDTH;                           // Coming from left (off-screen)
        default: return 0;
    }
}

float getInitialY(char road)
{
    switch (road) {
        case 'A': return -VEHICLE_HEIGHT;                          // Coming from top (off-screen)
        case 'B': return WINDOW_HEIGHT + VEHICLE_HEIGHT;           // Coming from bottom (off-screen)
        case 'C': return WINDOW_HEIGHT / 2 - LANE_WIDTH / 2 + 15;  // Coming from right
        case 'D': return WINDOW_HEIGHT / 2 + 15;                   // Coming from left
        default: return 0;
    }
}

float getStopPositionX(char road, int queuePosition)
{
    switch (road) {
        case 'A': return WINDOW_WIDTH / 2 - LANE_WIDTH / 2 + 15;
        case 'B': return WINDOW_WIDTH / 2 + 15;
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
        case 'C': return WINDOW_HEIGHT / 2 - LANE_WIDTH / 2 + 15;
        case 'D': return WINDOW_HEIGHT / 2 + 15;
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

    //set initial position (off-screen based on road)
    newNode->x = getInitialX(road);
    newNode->y = getInitialY(road);

    //set target position (stop line based on queue position)
    int queuePos = queue->size;
    newNode->targetX = getStopPositionX(road, queuePos);
    newNode->targetY = getStopPositionY(road, queuePos);


    if (queue->rear == NULL){
        queue->front = newNode;
        queue->rear = newNode;

    }else{
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
    queue->size++;
    SDL_Log("enqueue vehicle %s to road %c (Queue size: %d)", vehicleNumber, road, queue->size);
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

VehicleNode *peekQueue(Queue *queue){
    return queue->front;
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

void printMessageHelper(const char *message, int count)
{
    for (int i = 0; i < count; i++)
        printf("%s\n", message);
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
    // Update target positions for all vehicles in queue after dequeue
    VehicleNode *current = queue->front;
    int position = 0;
    while (current != NULL) {
        if (!current->hasCrossed) {
            current->targetX = getStopPositionX(current->road, position);
            current->targetY = getStopPositionY(current->road, position);
            current->isMoving = true;
        }
        current = current->next;
        position++;
    }
}

void updateVehicles(QueueData *queueData, float deltaTime){
    //Function to implement update the properties of vehicles for drawing
}

void drawVehicles(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData)
{
    // drawing vehicles for each lane
    // for lane A top and going down 
    VehicleNode *current = queueData->queueA->front;
    int yPos = 50;
    int count = 0;

    while (current != NULL && count < 10)
    {
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // blue
        SDL_Rect vehicleRect = {
            WINDOW_WIDTH / 2 - LANE_WIDTH / 2 + 5,
            yPos,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT
        };
        SDL_RenderFillRect(renderer, &vehicleRect);
        //displayText(renderer, font, current->vehicleNumber, vehicleRect.x + 5, vehicleRect.y + 2);
        
        current = current->next;
        yPos += VEHICLE_HEIGHT + 5;
        count++;

    }


    // for lane B bottom and going up
    current = queueData->queueB->front;
    yPos = WINDOW_HEIGHT - 50 - VEHICLE_HEIGHT;
    count = 0;
    while( current != NULL && count < 10){
        SDL_SetRenderDrawColor(renderer, 255, 0,0,255);//
        SDL_Rect vehicleRect = {
            WINDOW_WIDTH/2 + 5,
            yPos,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT
        };
        SDL_RenderFillRect(renderer, &vehicleRect);

        //displayText(renderer, font, current->vehicleNumber, vehicleRect.x + 5, vehicleRect.y + 2);
        current = current->next;
        yPos -= VEHICLE_HEIGHT + 5;
        count++;
    }

    // for lane C right and going left
    current = queueData->queueC->front;
    int xPos = WINDOW_WIDTH - 50 - VEHICLE_WIDTH;
    count = 0;
    while (current != NULL && count < 10)
    {
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Green
        SDL_Rect vehicleRect = {
            xPos,
            WINDOW_HEIGHT / 2 - LANE_WIDTH / 2,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT
        };
        SDL_RenderFillRect(renderer, &vehicleRect);
        
        //displayText(renderer, font, current->vehicleNumber, vehicleRect.x + 2, vehicleRect.y + 10);
        
        current = current->next;
        xPos -= (VEHICLE_WIDTH + 5);
        count++;
    }

    // for lane D left and going right
    current = queueData->queueD->front;
    xPos = 50;
    count = 0;
    while (current != NULL && count < 10)
    {
        SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255); // Yellow
        SDL_Rect vehicleRect = {
            xPos,
            WINDOW_HEIGHT / 2 + LANE_WIDTH / 2 - VEHICLE_HEIGHT,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT
        };
        SDL_RenderFillRect(renderer, &vehicleRect);
        
        //displayText(renderer, font, current->vehicleNumber, vehicleRect.x + 2, vehicleRect.y + 10);
        
        current = current->next;
        xPos += (VEHICLE_WIDTH + 5);
        count++;
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
    queueData.mutex = mutex;

    SharedData sharedData = {0, 0, &queueData, mutex};
 // 0 => all red


    TTF_Font *font = TTF_OpenFont(MAIN_FONT, 24);
    if (!font){
        SDL_Log("Failed to load font: %s", TTF_GetError());
    }
    
    // we need to create seprate long running thread for the queue processing and light

    // pthread_create(&tLight, NULL, refreshLight, &sharedData);
    pthread_create(&tQueue,NULL,checkQueue,&sharedData);
    pthread_create(&tReadFile,NULL,readAndParseFile,&queueData);
    // readAndParseFile();

    // Continue the UI thread
    bool running = true;
    while (running)
    {
        // Handle events first
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT)
                running = false;

        SDL_LockMutex(mutex);
        refreshLight(renderer, &sharedData, font);  // Only call once
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
    // pthread_kil
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
    // font init
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
    // if you have high resolution monitor 2K or 4K then scale
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

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

void drawArrow(SDL_Renderer *renderer, int x1, int y1, int x2, int y2, int x3, int y3)
{
    // Sort vertices by ascending Y (bubble sort approach)
    if (y1 > y2)
    {
        swap(&y1, &y2);
        swap(&x1, &x2);
    }
    if (y1 > y3)
    {
        swap(&y1, &y3);
        swap(&x1, &x3);
    }
    if (y2 > y3)
    {
        swap(&y2, &y3);
        swap(&x2, &x3);
    }

    // Compute slopes
    float dx1 = (y2 - y1) ? (float)(x2 - x1) / (y2 - y1) : 0;
    float dx2 = (y3 - y1) ? (float)(x3 - x1) / (y3 - y1) : 0;
    float dx3 = (y3 - y2) ? (float)(x3 - x2) / (y3 - y2) : 0;

    float sx1 = x1, sx2 = x1;

    // Fill first part (top to middle)
    for (int y = y1; y < y2; y++)
    {
        SDL_RenderDrawLine(renderer, (int)sx1, y, (int)sx2, y);
        sx1 += dx1;
        sx2 += dx2;
    }

    sx1 = x2;

    // Fill second part (middle to bottom)
    for (int y = y2; y <= y3; y++)
    {
        SDL_RenderDrawLine(renderer, (int)sx1, y, (int)sx2, y);
        sx1 += dx3;
        sx2 += dx2;
    }
}

void drawLightForB(SDL_Renderer *renderer, bool isRed)
{
    // draw light box
    SDL_SetRenderDrawColor(renderer, 150, 150, 150, 255);
    SDL_Rect lightBox = {400, 300, 50, 30};
    SDL_RenderFillRect(renderer, &lightBox);
    // draw light
    if (isRed)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // red
    else
        SDL_SetRenderDrawColor(renderer, 11, 156, 50, 255); // green
    SDL_Rect straight_Light = {405, 305, 20, 20};
    SDL_RenderFillRect(renderer, &straight_Light);
    drawArrow(renderer, 435, 305, 435, 305 + 20, 435 + 10, 305 + 10);
}

void drawRoadsAndLane(SDL_Renderer *renderer, TTF_Font *font)
{
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    // Vertical road

    SDL_Rect verticalRoad = {WINDOW_WIDTH / 2 - ROAD_WIDTH / 2, 0, ROAD_WIDTH, WINDOW_HEIGHT};
    SDL_RenderFillRect(renderer, &verticalRoad);

    // Horizontal road
    SDL_Rect horizontalRoad = {0, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2, WINDOW_WIDTH, ROAD_WIDTH};
    SDL_RenderFillRect(renderer, &horizontalRoad);
    // draw horizontal lanes
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    for (int i = 0; i <= 3; i++)
    {
        // Horizontal lanes
        SDL_RenderDrawLine(renderer,
                           0, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i,                                // x1,y1
                           WINDOW_WIDTH / 2 - ROAD_WIDTH / 2, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i // x2, y2
        );
        SDL_RenderDrawLine(renderer,
                           800, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i,
                           WINDOW_WIDTH / 2 + ROAD_WIDTH / 2, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 + LANE_WIDTH * i);
        // Vertical lanes
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
        case 0: // Lane A (top)
            x = WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 10;
            y = WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 40;
            break;
        case 1: // Lane B (bottom)
            x = WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 50;
            y = WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 10;
            break;
        case 2: // Lane C (right)
            x = WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 10;
            y = WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 10;
            break;
        case 3: // Lane D (left)
            x = WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 50;
            y = WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 40;
            break;
        default:
            return;
    }

    //Draw light box
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_Rect lightBox = {x, y, 40, 30};
    SDL_RenderFillRect(renderer, &lightBox);

    // draw red light
    if (!isGreen)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 100, 0, 0, 255);
    SDL_Rect redLight = {x + 5, y + 5, 12, 12};
    SDL_RenderFillRect(renderer, &redLight);

    //draw green light
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
    // display necessary text
    SDL_Color textColor = {0, 0, 0, 255}; // black color
    SDL_Surface *textSurface = TTF_RenderText_Solid(font, text, textColor);
    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_FreeSurface(textSurface);
    SDL_Rect textRect = {x, y, 0, 0};
    SDL_QueryTexture(texture, NULL, NULL, &textRect.w, &textRect.h);
    SDL_Log("DIM of SDL_Rect %d %d %d %d", textRect.x, textRect.y, textRect.h, textRect.w);
    // SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    // SDL_Log("TTF_Error: %s\n", TTF_GetError());
    SDL_RenderCopy(renderer, texture, NULL, &textRect);
    // SDL_Log("TTF_Error: %s\n", TTF_GetError());
}

void refreshLight(SDL_Renderer *renderer, SharedData *sharedData, TTF_Font *font)
{
    //clear screen
    SDL_SetRenderDrawColor(renderer, 0, 200, 0, 255);
    SDL_RenderClear(renderer);

    //draw roads and lanes
    drawRoadsAndLane(renderer, font);

    //draw traffic lights (activeLane is nextLight - 1, or -1 if all red)
    int activeLane = sharedData->nextLight - 1;
    drawAllTrafficLights(renderer, activeLane);
}


void *checkQueue(void *arg)
{
    SharedData *sharedData = (SharedData *)arg;
    QueueData *queueData = sharedData->queueData;
    while (1)
    {
        SDL_LockMutex(queueData->mutex);

        //to check if the lane A(aL2) priority
        int sizeA = getQueueSize(queueData->queueA);

        if (sizeA > PRIORITY_THRESHOLD_HIGH){
            queueData->priorityMode = 1;
            SDL_Log("Priority mode activated!! lane A has %d vehicles",sizeA);
        }else if(sizeA < PRIORITY_THRESHOLD_LOW && queueData->priorityMode == 1){
            queueData->priorityMode = 0;
            SDL_Log("Normal Mode continued!! lane A has %d vehicle",sizeA);
        }

        int laneToServe;
        int vehiclesToServe = 1;  // Initialize with default value

        if(queueData->priorityMode == 1){
            //priority mode serve lane A
            laneToServe = 0;
            vehiclesToServe = sizeA > 5 ? 5 : sizeA;  // Limit to 5 at a time
            SDL_Log("serving priority lane A with %d vehicles",vehiclesToServe);
        }else {
            //normal mode serve fairly
            int sizeB = getQueueSize(queueData->queueB);
            int sizeC = getQueueSize(queueData->queueC);
            int sizeD = getQueueSize(queueData->queueD);

            //changing lanes to serve
            laneToServe = queueData->currentLane;

            //get size of current lane
            int currentSize = 0;
            switch (laneToServe) {
                case 0: currentSize = sizeA; break;
                case 1: currentSize = sizeB; break;
                case 2: currentSize = sizeC; break;
                case 3: currentSize = sizeD; break;
            }
            //Calculate average vehicles to serve 
            int totalVehicles = sizeA + sizeB + sizeC + sizeD;
            int avgVehicles = (totalVehicles + 3) / 4;
            vehiclesToServe = (currentSize < avgVehicles) ? currentSize : avgVehicles;
            if (vehiclesToServe == 0) vehiclesToServe = 1;

            SDL_Log("normal mode service lane %d with %d vehicles (average: %d)", laneToServe, vehiclesToServe, avgVehicles);
            
            // move to next lane for next cycle 
            queueData->currentLane = (queueData->currentLane + 1) % 4;
        }

        SDL_UnlockMutex(queueData->mutex);

        //setting traffic light to red( all stop)
        sharedData->nextLight = 0;
        sleep(1);

        //setting traffic light to green for serving lane
        sharedData->nextLight = laneToServe + 1;

        //Process vehicles
        int greenLightTime = vehiclesToServe * TIME_PER_VEHICLE;
        if(greenLightTime > 0){
            SDL_Log("green light for lane %d for %d seconds", laneToServe, greenLightTime);
            for (int i = 0; i < vehiclesToServe; i++){
                SDL_LockMutex(queueData->mutex);
                Queue *queue = NULL;
                switch(laneToServe){
                    case 0: queue = queueData->queueA; break;  // Fixed: added space
                    case 1: queue = queueData->queueB; break;  // Fixed: added space
                    case 2: queue = queueData->queueC; break;  // Fixed: added space
                    case 3: queue = queueData->queueD; break;  // Fixed: added space
                }

                if (queue && getQueueSize(queue) > 0) {
                    VehicleNode *vehicle = dequeue(queue);
                    if (vehicle) {
                        SDL_Log("Vehicle %s passed through intersection from lane %c",
                                vehicle->vehicleNumber, 'A' + laneToServe);
                        free(vehicle);
                    }
                }

                SDL_UnlockMutex(queueData->mutex);
                sleep(TIME_PER_VEHICLE);
            }
        }
        else{
            sleep(2); //wait if no vehicles
        }
    }
    return NULL;
}

// you may need to pass the queue on this function for sharing the data
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

        //seek to last read position to avoid re-reading
        fseek(file, lastFilePos, SEEK_SET);

        char line[MAX_LINE_LENGTH];
        while (fgets(line, sizeof(line), file))
        {
            // Remove newline if present
            line[strcspn(line, "\n")] = 0;
            
            if (strlen(line) == 0) continue;  // Skip empty lines

            // Split using ':'
            char *vehicleNumber = strtok(line, ":");
            char *roadStr = strtok(NULL, ":"); // read next item resulted from split

            if (vehicleNumber && roadStr){
                char road = roadStr[0]; //getting first character
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
                
                SDL_UnlockMutex(queueData->mutex);  // Fixed: moved unlock here
            }
        }
        lastFilePos = ftell(file);
        fclose(file);
        sleep(1); // manage this time
    }
    return NULL;
}