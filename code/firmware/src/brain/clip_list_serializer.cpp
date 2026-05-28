#include "clip_list_serializer.h"
#include <ArduinoJson.h>

size_t buildClipListJson(const FhClip* clips, uint8_t count, char* buf, size_t buflen) {
    JsonDocument doc;
    JsonArray arr = doc["clips"].to<JsonArray>();
    for (uint8_t i = 0; i < count; i++) {
        JsonObject obj = arr.add<JsonObject>();
        obj["id"]   = i;
        obj["name"] = clips[i].name;
        obj["ms"]   = clips[i].duration_ms;
    }
    return serializeJson(doc, buf, buflen);
}
