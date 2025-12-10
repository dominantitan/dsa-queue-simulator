#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#define MSG_QUEUE_KEY 1234
#define MAX_TEXT 100

struct message {
    long msg_type;
    char vehicleQueue[MAX_TEXT];
};

int main() {
    key_t key = MSG_QUEUE_KEY;
    int msgid;
    struct message msg;
    
    msgid = msgget(key, 0666 | IPC_CREAT);
    if (msgid == -1) {
        perror("msgget failed");
        exit(1);
    }
    printf("Receiver is running... Waiting for messages.\n");
    while (1) {
        // Receive message (blocking call)
        if (msgrcv(msgid, &msg, sizeof(msg.vehicleQueue), 1, 0) == -1) {
            perror("msgrcv failed");
            exit(1);
        }
        printf("Received: %s\n", msg.vehicleQueue);
    }
    return 0;
}
