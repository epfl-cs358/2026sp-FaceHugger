#include <WebSocketsServer.h>
#include "network.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "shared/data.h"

WebSocketsServer webSocket = WebSocketsServer(81);

void initNetwork() {
    // 1. Explicitly set mode to Access Point
    WiFi.mode(WIFI_AP); 

    // 2. Configure the AP (SSID, Password, Channel, Hidden, Max Connections)
    // Using Channel 6 to avoid interference, allowing 4 simultaneous users
    bool success = WiFi.softAP("FaceHugger_Net", "12345678", 6, 0, 4);

    if (success) {
        Serial.println("\n======================================");
        Serial.println("ROSS (Robot OS) Network Online");
        Serial.print("SSID: FaceHugger_Net\nIP:   ");
        Serial.println(WiFi.softAPIP());
        Serial.println("======================================\n");
    }

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
        case CMD_TELEMETRY:
            Serial.printf("FSM state: %d, Battery voltage: %lf, In stabilization mode: %s\n", 
                (int)doc["s"], (float)doc["b"], (int)doc["a"] ? "true": "false");

            JsonArray dists = doc["d"];
            for(int i = 0; i < dists.size(); i++) {
                int d = dists[i];
                Serial.print("D");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(d);
                if (i < dists.size() - 1) Serial.print(" | ");
            }
            Serial.println();
            break;
    }
}

void updateNetwork() {
    webSocket.loop();
}