#include <WebSocketsServer.h>
#include "network.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "shared/data.h"
#include "../nervous_system/spinal_cord.h"
#include "../nervous_system/movements.h"

WebSocketsServer webSocket = WebSocketsServer(81);
extern SpinalCord spinalCord;

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
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] ❌ Event: Disconnected!\n", num);
            break;
            
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] ✅ Event: Connected from %s | URL: %s\n", num, ip.toString().c_str(), payload);
            break;
        }

        case WStype_TEXT:
            Serial.printf("[%u] 📩 Received Text: %s\n", num, payload);
            handleParsedMessage(payload);
            break;

        case WStype_ERROR:
            Serial.printf("[%u] ⚠️ Error Event occurred! Length: %u\n", num, length);
            break;
            
        case WStype_BIN:
            Serial.printf("[%u] 📦 Received Binary. Length: %u\n", num, length);
            break;
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
        case CMD_CALIBRATE: { 
            int channel = doc["id"] | 0;
            int angle = doc["a"] | 90; 

            spinalCord.applyCalibration(channel, angle);
            Serial.printf("Calibrating servo %d to %d", channel, angle);
            break;
        }
        case CMD_MOVE: {
            if (doc["g"].is<int>()) {
                int g = doc["g"];
                if (g >= GAIT_NONE && g <= GAIT_CRAB) {
                    GaitType requested = (GaitType)g;
                    if (requested != spinalCord.currentGait()) {
                        spinalCord.setGait(requested);
                    }
                }
            }
            Serial.printf("Moving -> X:%.2f Y:%.2f\n", (float)doc["x"], (float)doc["y"]);
            break;
        }
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