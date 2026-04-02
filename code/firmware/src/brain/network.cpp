#include <WebSocketsServer.h>
#include "network.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "shared/data.h"

WebSocketsServer webSocket = WebSocketsServer(81);

void initNetwork() {
    // Setup ESP32 as an Access Point
    WiFi.softAP("FaceHugger_Net", "12345678");
    Serial.println("WiFi AP Started: FaceHugger_Net");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        handleParsedMessage(payload);
    }
}

void handleParsedMessage(uint8_t * payload) {
    JsonDocument doc; // ArduinoJson 7 syntax
    DeserializationError error = deserializeJson(doc, payload);

    if (error) return;

    int type = doc["T"];

    switch (type) {
        case CMD_STATE:
            Serial.printf("State Change Request: %d\n", (int)doc["s"]);
            break;
        case CMD_CALIBRATE:
            Serial.printf("Calibrating Servo %d to Pulse %d\n", (int)doc["id"], (int)doc["p"]);
            break;
        case CMD_MOVE:
            Serial.printf("Moving -> X:%.2f Y:%.2f\n", (float)doc["x"], (float)doc["y"]);
            break;
    }
}

void updateNetwork() {
    webSocket.loop();
}