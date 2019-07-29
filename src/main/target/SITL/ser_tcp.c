#include "ser_tcp.h"
#include "build/build_config.h"
#include "common/utils.h"
#include "io/serial.h"
#include "platform.h"
#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

static const struct serialPortVTable ser_tcp_VTable;

void ser_tcp_init(ser_tcp_t *context) {

  int len;
  struct sockaddr_in servaddr, cli;

  context->sockfd = socket(AF_INET, SOCK_STREAM, 0);

  if (context->sockfd == -1) {
    printf("socket creation failed...\n");
    exit(0);
  } else
    printf("Socket successfully created..\n");

  bzero(&servaddr, sizeof(servaddr));

  // assign IP, PORT
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(context->tcp_port);

  int yes = 1;
  if (setsockopt(context->sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                 sizeof(int)) == -1) {
    perror("setsockopt");
  }
  // Binding newly created socket to given IP and verification
  if ((bind(context->sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) !=
      0) {
    printf("socket bind failed...\n");
    exit(0);
  } else
    printf("Socket successfully binded..\n");

  // Now server is ready to listen and verification
  if ((listen(context->sockfd, 5)) != 0) {
    printf("Listen failed...\n");
    exit(0);
  } else
    printf("Server listening..\n");
  len = sizeof(cli);

  // Accept the data packet from client and verification
  while (true) {
    context->connfd = accept(context->sockfd, (struct sockaddr *)&cli, &len);
    if (context->connfd < 0) {
      printf("server acccept failed...\n");
      exit(0);
    } else
      printf("server acccept the client...\n");
  }
}

static void *ser_tcp_thread(void *data) {
  ser_tcp_init((ser_tcp_t *)data);
  return NULL;
}

serialPort_t *ser_tcp_open(int id, serialReceiveCallbackPtr rxCallback,
                           void *rxCallbackData, uint32_t baudRate,
                           portMode_e mode, portOptions_e options) {
  ser_tcp_t *s = (ser_tcp_t *)malloc(sizeof(ser_tcp_t));

  if (!s)
    return NULL;

  s->tcp_port = 3333 + id;

  s->port.vTable = &ser_tcp_VTable;

  // common serial initialisation code should move to serialPort::init()
  s->port.rxBufferHead = s->port.rxBufferTail = 0;
  s->port.txBufferHead = s->port.txBufferTail = 0;
  s->port.rxBufferSize = RX_BUFFER_SIZE;
  s->port.txBufferSize = TX_BUFFER_SIZE;
  s->port.rxBuffer = s->rxBuffer;
  s->port.txBuffer = s->txBuffer;

  // callback works for IRQ-based RX ONLY
  s->port.rxCallback = rxCallback;
  s->port.rxCallbackData = rxCallbackData;
  s->port.mode = mode;
  s->port.baudRate = baudRate;
  s->port.options = options;

  pthread_mutex_init(&s->rxLock, NULL);
  pthread_mutex_init(&s->txLock, NULL);
  pthread_create(&s->worker, NULL, ser_tcp_thread, s);

  return (serialPort_t *)s;
}

uint32_t ser_tcp_totalRxBytesWaiting(const serialPort_t *instance) {
  ser_tcp_t *s = (ser_tcp_t *)instance;
  uint32_t count = 0;
  int bytes_available = 0;
  if (0 == ioctl(s->connfd, FIONREAD, &bytes_available)) {
    count = bytes_available;
  }
  return count;
}

uint32_t ser_tcp_totalTxBytesFree(const serialPort_t *instance) {
  return 64 * 1024;
}

bool ser_tcp_isTcpTransmitBufferEmpty(const serialPort_t *instance) {
  return true;
}

uint8_t ser_tcp_read(serialPort_t *instance) {
  uint8_t ch;
  ser_tcp_t *s = (ser_tcp_t *)instance;
  // pthread_mutex_lock(&s->rxLock);
  read(s->connfd, &ch, sizeof(ch));
  // pthread_mutex_unlock(&s->rxLock);
  return ch;
}

void ser_tcp_write(serialPort_t *instance, uint8_t ch) {
  ser_tcp_t *s = (ser_tcp_t *)instance;
  pthread_mutex_lock(&s->txLock);
  write(s->connfd, &ch, sizeof(ch));
  pthread_mutex_unlock(&s->txLock);
}

void ser_tcp_writeBuf(serialPort_t *instance, const void *data, int count) {
  ser_tcp_t *s = (ser_tcp_t *)instance;
  write(s->connfd, data, count);
}

void ser_tcp_beginWrite(serialPort_t *instance) {
  ser_tcp_t *s = (ser_tcp_t *)instance;
  pthread_mutex_lock(&s->txLock);
}

void ser_tcp_endWrite(serialPort_t *instance) {
  ser_tcp_t *s = (ser_tcp_t *)instance;
  pthread_mutex_unlock(&s->txLock);
}

static const struct serialPortVTable ser_tcp_VTable = {
    .serialWrite = ser_tcp_write,
    .serialTotalRxWaiting = ser_tcp_totalRxBytesWaiting,
    .serialTotalTxFree = ser_tcp_totalTxBytesFree,
    .serialRead = ser_tcp_read,
    .serialSetBaudRate = NULL,
    .isSerialTransmitBufferEmpty = ser_tcp_isTcpTransmitBufferEmpty,
    .setMode = NULL,
    .setCtrlLineStateCb = NULL,
    .setBaudRateCb = NULL,
    .writeBuf = ser_tcp_writeBuf,
    .beginWrite = ser_tcp_beginWrite,
    .endWrite = ser_tcp_endWrite,
};
