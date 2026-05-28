#include <unity.h>
#include <ArduinoJson.h>
#include "../../src/brain/clip_list_serializer.h"

void setUp(void) {}
void tearDown(void) {}

static const FhClipFrame dummy_frames[1] = {{ 0, {0,0,0,0,0,0,0,0,0,0,0,0} }};
static const FhClip mock_clips[2] = {
    { "wave",   dummy_frames, 1, 1208 },
    { "wiggle", dummy_frames, 1, 3000 },
};

void test_clip_list_json_structure(void) {
    char buf[256];
    size_t n = buildClipListJson(mock_clips, 2, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, n);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf);
    TEST_ASSERT_EQUAL(DeserializationError::Ok, err.code());

    JsonArray clips = doc["clips"].as<JsonArray>();
    TEST_ASSERT_EQUAL(2, clips.size());

    TEST_ASSERT_EQUAL_INT(0,         (int)clips[0]["id"]);
    TEST_ASSERT_EQUAL_STRING("wave", (const char*)clips[0]["name"]);
    TEST_ASSERT_EQUAL_INT(1208,      (int)clips[0]["ms"]);

    TEST_ASSERT_EQUAL_INT(1,            (int)clips[1]["id"]);
    TEST_ASSERT_EQUAL_STRING("wiggle",  (const char*)clips[1]["name"]);
    TEST_ASSERT_EQUAL_INT(3000,         (int)clips[1]["ms"]);
}

void test_clip_list_json_empty(void) {
    char buf[64];
    size_t n = buildClipListJson(nullptr, 0, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, n);

    JsonDocument doc;
    deserializeJson(doc, buf);
    TEST_ASSERT_EQUAL(0, (int)doc["clips"].as<JsonArray>().size());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_clip_list_json_structure);
    RUN_TEST(test_clip_list_json_empty);
    return UNITY_END();
}
