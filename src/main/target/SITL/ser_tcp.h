#ifndef SER_TCP_H
#define SER_TCP_H

#include <stdint.h>
#include "io/serial.h"
#include <pthread.h>

#define RX_BUFFER_SIZE    1400
#define TX_BUFFER_SIZE    1400


typedef struct {

    serialPort_t port;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint8_t txBuffer[TX_BUFFER_SIZE];
    pthread_mutex_t txLock;
    pthread_mutex_t rxLock;
    pthread_t worker;
    int sockfd;
    int tcp_port;
    int connfd;
} ser_tcp_t;

serialPort_t *ser_tcp_open(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);

#endif