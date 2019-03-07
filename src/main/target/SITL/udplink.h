/**
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */

#ifndef __UDPLINK_H
#define __UDPLINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*OnUdpData)(const char* data, uint32_t len);

typedef struct {
    int fd;
    struct sockaddr_in si;
    struct sockaddr_in recv;
    int port;
    char* addr;
    bool isServer;
    pthread_t thread;
    uint32_t timeout_ms;
    OnUdpData onData;
    char* buffer;
    uint32_t buffer_size;
    bool running;
} udpLink_t;


int udpInit(udpLink_t* link, const char* addr, int port, bool isServer);
int udpRecv(udpLink_t* link, void* data, size_t size, uint32_t timeout_ms);
int udpSend(udpLink_t* link, const void* data, size_t size);
int udpInitRecvThread(udpLink_t* link, const char* addr, int port, OnUdpData callback, uint32_t timeout_ms, char* buffer, uint32_t buffer_size);
void udpThreadStop(udpLink_t* link);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
