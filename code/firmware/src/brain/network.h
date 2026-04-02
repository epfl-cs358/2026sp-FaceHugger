#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>
#include <WebSocketsServer.h>

// Initializes WiFi Access Point and WebSocket Server
void initNetwork();

// Handles incoming WebSocket events (Connection, Data, Disconnection)
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// Parses JSON payload and executes logic
void handleParsedMessage(uint8_t * payload);

// Background task to be called in loop()
void updateNetwork();

extern WebSocketsServer webSocket;

#endif