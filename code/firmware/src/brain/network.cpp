#include <WebSocketsServer.h>
#include "network.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "shared/data.h"
#include "shared/config.h"
#include "../nervous_system/spinal_cord.h"
#include "../nervous_system/movements.h"
#include "../nervous_system/motion_math.h"   // clampPoseEaseMs (T:2 dur_ms)
#include "clip_list_serializer.h"
#include "../nervous_system/clips_all.h"

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
    }else{
        Serial.println("\n======================================");
        Serial.println("Did not succeed to create the access point \n");
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
            handleParsedMessage(num, payload);
            break;

        case WStype_ERROR:
            Serial.printf("[%u] ⚠️ Error Event occurred! Length: %u\n", num, length);
            break;
            
        case WStype_BIN:
            Serial.printf("[%u] 📦 Received Binary. Length: %u\n", num, length);
            break;
    }
}

void handleParsedMessage(uint8_t num, uint8_t * payload) {
    JsonDocument doc; // ArduinoJson 7 syntax
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.println("Failed to parse JSON");
        return;
    }

    int type = doc["T"];

    switch (type) {
        case CMD_STATE:
            if(doc.containsKey("s")){
                int newState = doc["s"];
                if(isValidStateCommand(newState)){
                    // Optional `dur_ms`: when present and > 0 on a pose state
                    // (REST/STAND), the firmware eases the pose over that many
                    // ms instead of snapping. Clamped to [0, POSE_EASE_MS_MAX]
                    // so a bad value can't park the robot in a multi-minute
                    // ease. Non-pose states ignore `dur_ms`.
                    uint32_t ease = doc["dur_ms"].is<uint32_t>()
                        ? clampPoseEaseMs(doc["dur_ms"].as<uint32_t>()) : 0u;
                    switch(newState){
                        case STATE_IDLE:
                            spinalCord.rest();
                            // Also clear any in-flight clip loop. The FSM
                            // transition alone left clipLoop_ + clipState_.phase
                            // alive; the app's page-blur stopMotion() would ease
                            // to neutral but the loop would resume on the next
                            // re-entry into STATE_ACTION.
                            spinalCord.stopClipPlayback();
                            break;
                        case STATE_WALK: spinalCord.walk(); break;
                        case STATE_ACTION: spinalCord.wallFlip(); break;
                        case STATE_REST:
                            if (ease > 0) spinalCord.relax(ease);
                            else          spinalCord.relax();   // flat / all-90 calibration
                            break;
                        case STATE_STAND:
                            if (ease > 0) spinalCord.stand(ease);
                            else          spinalCord.stand();   // standing / neutral
                            break;
                        default: break;                          // FAILSAFE: no-op (unchanged)
                    }
                }
            }
            break;
        case CMD_CALIBRATE: {
            if(doc.containsKey("id") && doc.containsKey("servo_id") && doc.containsKey("a")){
                int id = doc["id"];
                int servoId = doc["servo_id"];
                int angle = doc["a"];
                // Reject out-of-range indices before touching LEG_SERVO_CHANNEL[4][3] —
                // a malformed packet (id:7, servo_id:9, negatives) would otherwise read OOB.
                if (!isValidServoIndex(id, servoId)) {
                    Serial.printf("[WARN] T:4 ignored: id=%d servo_id=%d out of range\n",
                                  id, servoId);
                    break;
                }
                //create a mapping between the channels and leg servo id
                uint8_t channel = LEG_SERVO_CHANNEL[id][servoId];
                spinalCord.applyCalibration(channel, angle);
                Serial.printf("Calibrating servo %d to %d", channel, angle);
            }
            break;  // always break the case — a malformed calibrate is a no-op,
                    // NOT a fall-through into CMD_MOVE (which would start walking).
        }

        case CMD_MOVE: {
            // Extract the direction string
            String dir = doc["dir"] | "";
            
            spinalCord.walk(); 
            
            
            // Send the intent to the 4-Phase Engine
            spinalCord.processCommand(dir);
            Serial.printf("Move -> Dir: %s", dir.c_str());
            break;
        }
        
        case CMD_GAIT_MODE: {
            if (doc["g"].is<int>()) {
                int g = doc["g"];
                if (g >= GAIT_NONE && g <= GAIT_CRAB) {
                    GaitType requested = (GaitType)g;
                    if (requested != spinalCord.currentGait()) {
                        spinalCord.setGait(requested);
                    }
                }
            }
            break;
        }
        case CMD_ACTION_SELECTION: {
            if(doc.containsKey("a")){
                int a = doc["a"];
                if(a == INVERT_ROBOT){
                    spinalCord.invertRobot();
                }
            }
            break;
        }
        case CMD_SET_INVERT: {
            if (!doc["inverted"].is<bool>()) {
                Serial.println("[WARN] T:9 ignored: 'inverted' key missing or not bool");
                break;
            }
            spinalCord.setInverted(doc["inverted"].as<bool>());
            break;
        }
        case CMD_PLAY_CLIP: {
            if (doc["c"].is<int>()) {
                int c = doc["c"];
                // Optional "loop": true replays the clip until another motion
                // command preempts it (default false = play once).
                bool loop = doc["loop"].is<bool>() && doc["loop"].as<bool>();
                if (c >= 0 && c < 256) {
                    spinalCord.playClip((uint8_t)c, loop);
                }
            }
            break;
        }
        case CMD_LIST_CLIPS: {
            char buf[512];
            buildClipListJson(FH_CLIPS, FH_CLIP_COUNT, buf, sizeof(buf));
            webSocket.sendTXT(num, buf);
            break;
        }
        case CMD_SET_SMOOTHING: {
            // {T:11, a:<0..1>} — runtime clip-playback smoothing (EMA alpha).
            // Higher = smoother but laggier; firmware clamps to a safe range.
            if (doc["a"].is<float>()) {
                spinalCord.setClipSmoothing(doc["a"].as<float>());
            }
            break;
        }
        case CMD_TELEMETRY: { //this is the robot that sends it
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
}

void updateNetwork() {
    webSocket.loop();
}