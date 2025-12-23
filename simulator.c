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

// checkQueue constants
#define PRIORITY_THRESHOLD_HIGH 10
#define PRIORITY_THRESHOLD_LOW 5
#define TIME_PER_VEHICLE 1 // seconds per vehicle

// vehicle box dimensions
#define VEHICLE_WIDTH 40
#define VEHICLE_HEIGHT 20

const char *VEHICLE_FILE = "vehicles.data";

// Forward declaration
struct QueueData;

typedef struct
{
    int currentLight;
    int nextLight;
    struct QueueData *queueData;
} SharedData;

// Node for queue
typedef struct VehicleNode
{
    char vehicleNumber[10];
    char road;
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
    int currentLane;
    int priorityMode;
    SDL_mutex *mutex;
} QueueData;

// Function declarations
bool initializeSDL(SDL_Window **window, SDL_Renderer **renderer);
void drawRoadsAndLane(SDL_Renderer *renderer, TTF_Font *font);
void displayText(SDL_Renderer *renderer, TTF_Font *font, char *text, int x, int y);
void drawLightForB(SDL_Renderer *renderer, bool isRed);
void drawLightForA(SDL_Renderer *renderer, bool isRed);
void drawLightForC(SDL_Renderer *renderer, bool isRed);
void drawLightForD(SDL_Renderer *renderer, bool isRed);
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
void drawArrow(SDL_Renderer *renderer, int x1, int y1, int x2, int y2, int x3, int y3);
void swap(int *a, int *b);
void drawQueueStatus(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData);

// Queue implementations
void initQueue(Queue *queue)
{
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
}

void enqueue(Queue *queue, const char *vehicleNumber, char road)
{
    VehicleNode *newNode = (VehicleNode *)malloc(sizeof(VehicleNode));
    if (!newNode)
    {
        SDL_Log("Memory allocation failed");
        return;
    }

    strncpy(newNode->vehicleNumber, vehicleNumber, sizeof(newNode->vehicleNumber) - 1);
    newNode->vehicleNumber[sizeof(newNode->vehicleNumber) - 1] = '\0';
    newNode->road = road;
    newNode->next = NULL;

    if (queue->rear == NULL)
    {
        queue->front = newNode;
        queue->rear = newNode;
    }
    else
    {
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
    queue->size++;
    SDL_Log("Enqueued vehicle %s to road %c (Queue size: %d)", vehicleNumber, road, queue->size);
}

VehicleNode *dequeue(Queue *queue)
{
    if (queue->front == NULL)
    {
        return NULL;
    }
    VehicleNode *temp = queue->front;
    queue->front = queue->front->next;

    if (queue->front == NULL)
    {
        queue->rear = NULL;
    }

    queue->size--;
    SDL_Log("Dequeued vehicle %s from road %c (Queue size: %d)", temp->vehicleNumber, temp->road, queue->size);
    return temp;
}

int getQueueSize(Queue *queue)
{
    return queue->size;
}

VehicleNode *peekQueue(Queue *queue)
{
    return queue->front;
}

void freeQueue(Queue *queue)
{
    VehicleNode *current = queue->front;
    while (current != NULL)
    {
        VehicleNode *temp = current;
        current = current->next;
        free(temp);
    }
    queue->front = NULL;
    queue->rear = NULL;
    queue->size = 0;
}

void drawVehicles(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData)
{
    // Lane A (top, going down) - vehicles queue from top
    VehicleNode *current = queueData->queueA->front;
    int yPos = 50;
    int count = 0;

    while (current != NULL && count < 8)
    {
        SDL_SetRenderDrawColor(renderer, 0, 100, 255, 255); // Blue
        SDL_Rect vehicleRect = {
            WINDOW_WIDTH / 2 - LANE_WIDTH / 2 + 5,
            yPos,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT};
        SDL_RenderFillRect(renderer, &vehicleRect);

        current = current->next;
        yPos += VEHICLE_HEIGHT + 5;
        count++;
    }

    // Lane B (bottom, going up) - vehicles queue from bottom
    current = queueData->queueB->front;
    yPos = WINDOW_HEIGHT - 80;
    count = 0;
    while (current != NULL && count < 8)
    {
        SDL_SetRenderDrawColor(renderer, 255, 100, 0, 255); // Orange
        SDL_Rect vehicleRect = {
            WINDOW_WIDTH / 2 + 5,
            yPos,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT};
        SDL_RenderFillRect(renderer, &vehicleRect);

        current = current->next;
        yPos -= (VEHICLE_HEIGHT + 5);
        count++;
    }

    // Lane C (right, going left) - vehicles queue from right
    current = queueData->queueC->front;
    int xPos = WINDOW_WIDTH - 80;
    count = 0;
    while (current != NULL && count < 8)
    {
        SDL_SetRenderDrawColor(renderer, 0, 200, 100, 255); // Green
        SDL_Rect vehicleRect = {
            xPos,
            WINDOW_HEIGHT / 2 - LANE_WIDTH / 2 + 5,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT};
        SDL_RenderFillRect(renderer, &vehicleRect);

        current = current->next;
        xPos -= (VEHICLE_WIDTH + 5);
        count++;
    }

    // Lane D (left, going right) - vehicles queue from left
    current = queueData->queueD->front;
    xPos = 40;
    count = 0;
    while (current != NULL && count < 8)
    {
        SDL_SetRenderDrawColor(renderer, 200, 200, 0, 255); // Yellow
        SDL_Rect vehicleRect = {
            xPos,
            WINDOW_HEIGHT / 2 + 5,
            VEHICLE_WIDTH,
            VEHICLE_HEIGHT};
        SDL_RenderFillRect(renderer, &vehicleRect);

        current = current->next;
        xPos += (VEHICLE_WIDTH + 5);
        count++;
    }
}

void drawQueueStatus(SDL_Renderer *renderer, TTF_Font *font, QueueData *queueData)
{
    char status[50];

    // Display queue sizes
    snprintf(status, sizeof(status), "A:%d", getQueueSize(queueData->queueA));
    displayText(renderer, font, status, 360, 30);

    snprintf(status, sizeof(status), "B:%d", getQueueSize(queueData->queueB));
    displayText(renderer, font, status, 420, 760);

    snprintf(status, sizeof(status), "C:%d", getQueueSize(queueData->queueC));
    displayText(renderer, font, status, 750, 360);

    snprintf(status, sizeof(status), "D:%d", getQueueSize(queueData->queueD));
    displayText(renderer, font, status, 10, 420);

    // Display priority mode status
    if (queueData->priorityMode)
    {
        displayText(renderer, font, "PRIORITY MODE", 320, 5);
    }
    else
    {
        displayText(renderer, font, "NORMAL MODE", 330, 5);
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

    // Initialize queue data
    QueueData queueData;
    queueData.queueA = (Queue *)malloc(sizeof(Queue));
    queueData.queueB = (Queue *)malloc(sizeof(Queue));
    queueData.queueC = (Queue *)malloc(sizeof(Queue));
    queueData.queueD = (Queue *)malloc(sizeof(Queue));

    initQueue(queueData.queueA);
    initQueue(queueData.queueB);
    initQueue(queueData.queueC);
    initQueue(queueData.queueD);

    queueData.currentLane = 0;
    queueData.priorityMode = 0;
    queueData.mutex = mutex;

    SharedData sharedData = {0, 0, NULL};
    sharedData.queueData = &queueData;

    TTF_Font *font = TTF_OpenFont(MAIN_FONT, 24);
    if (!font)
    {
        SDL_Log("Failed to load font: %s", TTF_GetError());
        return -1;
    }

    // Create threads
    pthread_create(&tQueue, NULL, checkQueue, &sharedData);
    pthread_create(&tReadFile, NULL, readAndParseFile, &queueData);

    // Main UI loop
    bool running = true;
    while (running)
    {
        // Clear screen
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        // Draw roads
        drawRoadsAndLane(renderer, font);

        // Draw traffic lights
        refreshLight(renderer, &sharedData);

        // Draw vehicles
        SDL_LockMutex(mutex);
        drawVehicles(renderer, font, &queueData);
        drawQueueStatus(renderer, font, &queueData);
        SDL_UnlockMutex(mutex);

        // Present renderer
        SDL_RenderPresent(renderer);

        // Handle events
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }

        SDL_Delay(100); // ~10 FPS for smoother updates
    }

    // Cleanup
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
        SDL_Log("SDL init failed: %s", SDL_GetError());
        return false;
    }
    if (TTF_Init() < 0)
    {
        SDL_Log("TTF init failed: %s", TTF_GetError());
        return false;
    }

    *window = SDL_CreateWindow("Traffic Junction Simulator",
                               SDL_WINDOWPOS_CENTERED,
                               SDL_WINDOWPOS_CENTERED,
                               WINDOW_WIDTH,
                               WINDOW_HEIGHT,
                               SDL_WINDOW_SHOWN);
    if (!*window)
    {
        SDL_Log("Window creation failed: %s", SDL_GetError());
        return false;
    }

    *renderer = SDL_CreateRenderer(*window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetScale(*renderer, SCALE, SCALE);

    if (!*renderer)
    {
        SDL_Log("Renderer creation failed: %s", SDL_GetError());
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
    if (y1 > y2)
    {
        swap(&x1, &x2);
        swap(&y1, &y2);
    }
    if (y1 > y3)
    {
        swap(&x1, &x3);
        swap(&y1, &y3);
    }
    if (y2 > y3)
    {
        swap(&x2, &x3);
        swap(&y2, &y3);
    }

    float dx1 = (y2 - y1) ? (float)(x2 - x1) / (y2 - y1) : 0;
    float dx2 = (y3 - y1) ? (float)(x3 - x1) / (y3 - y1) : 0;
    float dx3 = (y3 - y2) ? (float)(x3 - x2) / (y3 - y2) : 0;

    float sx1 = x1, sx2 = x1;

    for (int y = y1; y < y2; y++)
    {
        SDL_RenderDrawLine(renderer, (int)sx1, y, (int)sx2, y);
        sx1 += dx1;
        sx2 += dx2;
    }

    sx1 = x2;

    for (int y = y2; y <= y3; y++)
    {
        SDL_RenderDrawLine(renderer, (int)sx1, y, (int)sx2, y);
        sx1 += dx3;
        sx2 += dx2;
    }
}

// Traffic lights for each road
void drawLightForA(SDL_Renderer *renderer, bool isRed)
{
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_Rect lightBox = {WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 60, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 40, 50, 30};
    SDL_RenderFillRect(renderer, &lightBox);

    if (isRed)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

    SDL_Rect light = {lightBox.x + 15, lightBox.y + 5, 20, 20};
    SDL_RenderFillRect(renderer, &light);
}

void drawLightForB(SDL_Renderer *renderer, bool isRed)
{
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_Rect lightBox = {WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 10, WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 10, 50, 30};
    SDL_RenderFillRect(renderer, &lightBox);

    if (isRed)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

    SDL_Rect light = {lightBox.x + 15, lightBox.y + 5, 20, 20};
    SDL_RenderFillRect(renderer, &light);
}

void drawLightForC(SDL_Renderer *renderer, bool isRed)
{
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_Rect lightBox = {WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 10, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 40, 50, 30};
    SDL_RenderFillRect(renderer, &lightBox);

    if (isRed)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

    SDL_Rect light = {lightBox.x + 15, lightBox.y + 5, 20, 20};
    SDL_RenderFillRect(renderer, &light);
}

void drawLightForD(SDL_Renderer *renderer, bool isRed)
{
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    SDL_Rect lightBox = {WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 60, WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 10, 50, 30};
    SDL_RenderFillRect(renderer, &lightBox);

    if (isRed)
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    else
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);

    SDL_Rect light = {lightBox.x + 15, lightBox.y + 5, 20, 20};
    SDL_RenderFillRect(renderer, &light);
}

void drawRoadsAndLane(SDL_Renderer *renderer, TTF_Font *font)
{
    // Draw roads (gray)
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);

    // Vertical road
    SDL_Rect verticalRoad = {WINDOW_WIDTH / 2 - ROAD_WIDTH / 2, 0, ROAD_WIDTH, WINDOW_HEIGHT};
    SDL_RenderFillRect(renderer, &verticalRoad);

    // Horizontal road
    SDL_Rect horizontalRoad = {0, WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2, WINDOW_WIDTH, ROAD_WIDTH};
    SDL_RenderFillRect(renderer, &horizontalRoad);

    // Draw lane dividers (white dashed lines)
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    // Vertical lane dividers
    for (int y = 0; y < WINDOW_HEIGHT; y += 30)
    {
        // Skip junction area
        if (y > WINDOW_HEIGHT / 2 - ROAD_WIDTH / 2 - 20 && y < WINDOW_HEIGHT / 2 + ROAD_WIDTH / 2 + 20)
            continue;

        SDL_RenderDrawLine(renderer,
                           WINDOW_WIDTH / 2 - LANE_WIDTH / 2, y,
                           WINDOW_WIDTH / 2 - LANE_WIDTH / 2, y + 15);
        SDL_RenderDrawLine(renderer,
                           WINDOW_WIDTH / 2 + LANE_WIDTH / 2, y,
                           WINDOW_WIDTH / 2 + LANE_WIDTH / 2, y + 15);
    }

    // Horizontal lane dividers
    for (int x = 0; x < WINDOW_WIDTH; x += 30)
    {
        // Skip junction area
        if (x > WINDOW_WIDTH / 2 - ROAD_WIDTH / 2 - 20 && x < WINDOW_WIDTH / 2 + ROAD_WIDTH / 2 + 20)
            continue;

        SDL_RenderDrawLine(renderer,
                           x, WINDOW_HEIGHT / 2 - LANE_WIDTH / 2,
                           x + 15, WINDOW_HEIGHT / 2 - LANE_WIDTH / 2);
        SDL_RenderDrawLine(renderer,
                           x, WINDOW_HEIGHT / 2 + LANE_WIDTH / 2,
                           x + 15, WINDOW_HEIGHT / 2 + LANE_WIDTH / 2);
    }

    // Road labels
    displayText(renderer, font, "A", WINDOW_WIDTH / 2 - 10, 50);
    displayText(renderer, font, "B", WINDOW_WIDTH / 2 - 10, WINDOW_HEIGHT - 70);
    displayText(renderer, font, "C", WINDOW_WIDTH - 70, WINDOW_HEIGHT / 2 - 10);
    displayText(renderer, font, "D", 50, WINDOW_HEIGHT / 2 - 10);
}

void displayText(SDL_Renderer *renderer, TTF_Font *font, char *text, int x, int y)
{
    SDL_Color textColor = {0, 0, 0, 255};
    SDL_Surface *textSurface = TTF_RenderText_Solid(font, text, textColor);
    if (!textSurface)
        return;

    SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, textSurface);
    SDL_FreeSurface(textSurface);

    if (!texture)
        return;

    SDL_Rect textRect = {x, y, 0, 0};
    SDL_QueryTexture(texture, NULL, NULL, &textRect.w, &textRect.h);
    SDL_RenderCopy(renderer, texture, NULL, &textRect);
    SDL_DestroyTexture(texture);
}

void refreshLight(SDL_Renderer *renderer, SharedData *sharedData)
{
    int currentLight = sharedData->currentLight;

    // Draw all lights - red by default, green for active lane
    drawLightForA(renderer, currentLight != 1);
    drawLightForB(renderer, currentLight != 2);
    drawLightForC(renderer, currentLight != 3);
    drawLightForD(renderer, currentLight != 4);
}

void *checkQueue(void *arg)
{
    SharedData *sharedData = (SharedData *)arg;
    QueueData *queueData = sharedData->queueData;

    while (1)
    {
        SDL_LockMutex(queueData->mutex);

        int sizeA = getQueueSize(queueData->queueA);
        int sizeB = getQueueSize(queueData->queueB);
        int sizeC = getQueueSize(queueData->queueC);
        int sizeD = getQueueSize(queueData->queueD);

        // Check priority mode
        if (sizeA > PRIORITY_THRESHOLD_HIGH)
        {
            queueData->priorityMode = 1;
            SDL_Log("PRIORITY MODE - Lane A has %d vehicles", sizeA);
        }
        else if (sizeA < PRIORITY_THRESHOLD_LOW && queueData->priorityMode == 1)
        {
            queueData->priorityMode = 0;
            SDL_Log("NORMAL MODE RESUMED");
        }

        int laneToServe;
        int vehiclesToServe;

        if (queueData->priorityMode == 1)
        {
            laneToServe = 0; // Lane A
            vehiclesToServe = sizeA > 5 ? 5 : sizeA;
        }
        else
        {
            // Round-robin normal mode
            laneToServe = queueData->currentLane;

            int sizes[] = {sizeA, sizeB, sizeC, sizeD};
            int totalVehicles = sizeA + sizeB + sizeC + sizeD;
            int avgVehicles = (totalVehicles > 0) ? (totalVehicles + 3) / 4 : 1;

            vehiclesToServe = (sizes[laneToServe] < avgVehicles) ? sizes[laneToServe] : avgVehicles;
            if (vehiclesToServe > 5)
                vehiclesToServe = 5;

            queueData->currentLane = (queueData->currentLane + 1) % 4;
        }

        SDL_UnlockMutex(queueData->mutex);

        // Set all lights red first
        sharedData->currentLight = 0;
        sleep(1);

        // Set green light for serving lane
        sharedData->currentLight = laneToServe + 1;
        SDL_Log("Green light for lane %d, serving %d vehicles", laneToServe, vehiclesToServe);

        // Process vehicles
        for (int i = 0; i < vehiclesToServe; i++)
        {
            SDL_LockMutex(queueData->mutex);

            Queue *queue = NULL;
            switch (laneToServe)
            {
            case 0:
                queue = queueData->queueA;
                break;
            case 1:
                queue = queueData->queueB;
                break;
            case 2:
                queue = queueData->queueC;
                break;
            case 3:
                queue = queueData->queueD;
                break;
            }

            VehicleNode *vehicle = dequeue(queue);
            if (vehicle)
            {
                SDL_Log("Vehicle %s passed through", vehicle->vehicleNumber);
                free(vehicle);
            }

            SDL_UnlockMutex(queueData->mutex);
            sleep(TIME_PER_VEHICLE);
        }

        if (vehiclesToServe == 0)
        {
            sleep(2);
        }
    }
    return NULL;
}

void *readAndParseFile(void *arg)
{
    QueueData *queueData = (QueueData *)arg;
    long lastPosition = 0;

    while (1)
    {
        FILE *file = fopen(VEHICLE_FILE, "r");
        if (!file)
        {
            SDL_Log("Waiting for %s...", VEHICLE_FILE);
            sleep(2);
            continue;
        }

        // Move to last read position
        fseek(file, lastPosition, SEEK_SET);

        char line[MAX_LINE_LENGTH];
        while (fgets(line, sizeof(line), file))
        {
            line[strcspn(line, "\n")] = 0;
            line[strcspn(line, "\r")] = 0;

            if (strlen(line) == 0)
                continue;

            char *vehicleNumber = strtok(line, ":");
            char *roadStr = strtok(NULL, ":");

            if (vehicleNumber && roadStr)
            {
                char road = roadStr[0];

                SDL_LockMutex(queueData->mutex);

                switch (road)
                {
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

        lastPosition = ftell(file);
        fclose(file);
        sleep(1);
    }
    return NULL;
}