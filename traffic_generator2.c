#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <time.h>
#include <unistd.h>

#define MSG_QUEUE_KEY 1234 // this is uniqe key to perform IPC
#define MAX_TEXT 100

struct message {
    long msg_type;
    char vehicleQueue[MAX_TEXT];
};

// Generate a random vehicle number
// Format:   <2 alpha><1 digit><2 alpha><3 digit>
void generateVehicleNumber(char* buffer) {
    buffer[0] = 'A' + rand() % 26;
    buffer[1] = 'A' + rand() % 26;
    buffer[2] = '0' + rand() % 10;
    buffer[3] = 'A' + rand() % 26;
    buffer[4] = 'A' + rand() % 26;
    buffer[5] = '0' + rand() % 10;
    buffer[6] = '0' + rand() % 10;
    buffer[7] = '0' + rand() % 10;
    buffer[8] = '\0';
}

// Generate a random lane
char generateLane() {
    char lanes[] = {'A', 'B', 'C', 'D'};
    return lanes[rand() % 4];
}

int main() {
    key_t key = MSG_QUEUE_KEY;
    int msgid;
    struct message msg;

    // Create a message queue
    msgid = msgget(key, 0666 | IPC_CREAT);
    if (msgid == -1) {
        perror("msgget failed");
        exit(1);
    }

    srand(time(NULL));

    while (1) {
        char vehicle[9];
        generateVehicleNumber(vehicle);
        char lane = generateLane();
        msg.msg_type = 1;
        snprintf(msg.vehicleQueue, MAX_TEXT, "%s:%c", vehicle, lane);

        // Send message
        if (msgsnd(msgid, &msg, sizeof(msg.vehicleQueue), 0) == -1) {
            perror("msgsnd failed");
            exit(1);
        }

        printf("New vehicle added: %s\n", msg.vehicleQueue);
        sleep(1);
    }
    return 0;
}