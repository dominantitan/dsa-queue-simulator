#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

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
#define TIME_PER_VEHICLE 1  // seconds per vehicle

//vehicle box dimensions
#define VEHICLE_WIDTH 40;
#define VEHICLE_HEIGHT 20;



const char *VEHICLE_FILE = "vehicles.data";

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
    char vehicleNumber[10];   // unique id for the vehicle
    char road;                // road the vehicle is in
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
} QueueData;

// Function declarations
bool initializeSDL(SDL_Window **window, SDL_Renderer **renderer);
void drawRoadsAndLane(SDL_Renderer *renderer, TTF_Font *font);
void displayText(SDL_Renderer *renderer, TTF_Font *font, char *text, int x, int y);
void drawLightForB(SDL_Renderer *renderer, bool isRed);
void refreshLight(SDL_Renderer *renderer, SharedData *sharedData);
void *checkQueue(void *arg);
void *readAndParseFile(void *arg);
void initQueue(Queue *queue);
void enqueue(Queue *queue, const char *vehicleNumber, char road);
VehicleNode *dequeue(Queue *queue);
int getQueueSize(Queue *queue);
VehicleNode *peekQueue(Queue *queue);
void freeQueue(Queue *queue);
void drawVehicles(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData);

void initQueue(Queue *queue){
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
}

void enqueue(Queue *queue,const char *vehicleNumber,char road)
{
    VehicleNode *newNode = (VehicleNode *)malloc(sizeof(VehicleNode));
    if(!newNode){
        return;
    }

    strncpy(newNode->vehicleNumber,vehicleNumber,sizeof(newNode->vehicleNumber)-1);
    newNode->vehicleNumber[sizeof(newNode->vehicleNumber)-1] = '\0';
    newNode->road = road;
    newNode->next = NULL;

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

    SharedData sharedData = {0, 0}; // 0 => all red
    sharedData.queueData = &queueData;


    TTF_Font *font = TTF_OpenFont(MAIN_FONT, 24);
    if (!font)
        SDL_Log("Failed to load font: %s", TTF_GetError());

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    drawRoadsAndLane(renderer, font);
    // drawLightForB(renderer, false);
    SDL_RenderPresent(renderer);

    // we need to create seprate long running thread for the queue processing and light

    // pthread_create(&tLight, NULL, refreshLight, &sharedData);
    pthread_create(&tQueue,NULL,checkQueue,&sharedData);
    pthread_create(&tReadFile,NULL,readAndParseFile,&queueData);
    // readAndParseFile();

    // Continue the UI thread
    bool running = true;
    while (running)
    {
        // update light
        refreshLight(renderer, &sharedData);

        SDL_LockMutex(mutex);
        drawVehicles(renderer,font,&queueData);
        SDL_UnlockMutex(mutex);
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT)
                running = false;

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
    SDL_SetRenderDrawColor(renderer, 211, 211, 211, 255);
    // Vertical road

    SDL_Rect verticalRoad = {WINDOW_WIDTH / 2 - ROAD_WIDTH / 2, 0, ROAD_WIDTH, WINDOW_HEIGHT};
    SDL_RenderFillRect(renderer, &verticalRoad);

    // Horizontal road
    SDL_Rect horizontalRoad = {0, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2, WINDOW_WIDTH, ROAD_WIDTH};
    SDL_RenderFillRect(renderer, &horizontalRoad);
    // draw horizontal lanes
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
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

void refreshLight(SDL_Renderer *renderer, SharedData *sharedData)
{
    if (sharedData->nextLight == sharedData->currentLight)
        return; // early return

    if (sharedData->nextLight == 0)
    { // trun off all lights
        drawLightForB(renderer, false);
    }
    if (sharedData->nextLight == 2)
        drawLightForB(renderer, true);
    else
        drawLightForB(renderer, false);
    SDL_RenderPresent(renderer);
    printf("Light of queue updated from %d to %d\n", sharedData->currentLight, sharedData->nextLight);
    // update the light
    sharedData->currentLight = sharedData->nextLight;
    fflush(stdout);
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
        int vehiclesToServe;

        if(queueData->priorityMode == 1){
            //priority mode serve lane A
            laneToServe = 0;
            vehiclesToServe = sizeA;
            SDL_Log("serving priority lane A with %d vehicles",vehiclesToServe);
        }else {
            //normal mode serve fairly
            int sizeB = getQueueSize(queueData->queueB);
            int sizeC = getQueueSize(queueData->queueC);
            int sizeD = getQueueSize(queueData->queueD);

            //changing lanes to serve
            laneToServe = queueData->currentLane;

            //get size of current lane
            Queue *currentQueue = NULL;
            switch(laneToServe){
                case 0: currentQueue = queueData->queueA;
                break;
                case 1: currentQueue = queueData->queueB;
                break;
                case 2: currentQueue = queueData->queueC;
                break;
                case 3: currentQueue = queueData->queueD;
                break;
                
            }
            //Calculate average vehicles to serve 
            int totalVehicles = sizeA + sizeB + sizeC + sizeD;
            int avgVehicles = (totalVehicles+3)/4;

            SDL_Log("normal mode service lane %d with %d vehicles (average; %d)",laneToServe,vehiclesToServe, avgVehicles);
            
            // move tot next lane for next cycle 
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
            SDL_Log("green light for lane %d for %d seconds",laneToServe,greenLightTime);
            for (int i = 0; i < vehiclesToServe;i++){
                SDL_LockMutex(queueData->mutex);
                Queue *queue = NULL;
                switch(laneToServe){
                    case0: queue = queueData->queueA;
                    break;
                
                    case1: queue = queueData->queueB;
                    break;

                    case2: queue = queueData->queueC;
                    break;

                    case3: queue = queueData->queueD;
                    break;
                }

                SDL_UnlockMutex(queueData->mutex);
                sleep(TIME_PER_VEHICLE);
            }
        }
        else{
            sleep(2);
        }
    }
    return NULL;
}

// you may need to pass the queue on this function for sharing the data
void *readAndParseFile(void *arg)
{
    QueueData *queueData = (QueueData *)arg;
    while (1)
    {
        FILE *file = fopen(VEHICLE_FILE, "r");
        if (!file)
        {
            SDl_Log("waiting for vehicle file...");
            sleep(2);
            continue;
        }

        char line[MAX_LINE_LENGTH];
        while (fgets(line, sizeof(line), file))
        {
            // Remove newline if present
            line[strcspn(line, "\n")] = 0;

            // Split using ':'
            char *vehicleNumber = strtok(line, ":");
            char *roadStr = strtok(NULL, ":"); // read next item resulted from split

            if (vehicleNumber && roadStr){
                char road = roadStr[0]; //getting first character
                SDL_LockMutex(queueData->mutex);

                switch (road){
                    case 'A':
                        enqueue(queueData->queueA,vehicleNumber, road);
                        break;
                    case 'B':
                        enqueue(queueData->queueB,vehicleNumber, road);
                        break;
                    case 'C':
                        enqueue(queueData->queueC,vehicleNumber, road);
                        break;
                    case 'D':
                        enqueue(queueData->queueD,vehicleNumber, road);
                        break;
                    default:
                        SDL_Log("Unknown road: %c",road);

                }

            }else{
                        SDL_UnlockMutex(queueData->mutex);
            }
        }
        fclose(file);
        sleep(2); // manage this time
    }
    return NULL;
}