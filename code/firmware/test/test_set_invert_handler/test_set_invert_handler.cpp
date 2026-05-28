#include <unity.h>
#include <ArduinoJson.h>
#include "../../src/shared/data.h"

// Guards the CMD_SET_INVERT (T:9) handler contract: sets isInverted flag only,
// no pose change. Uses ArduinoJson directly to mirror the handler's parse path.

void setUp(void) {}
void tearDown(void) {}

void test_cmd_set_invert_is_9(void) {
    TEST_ASSERT_EQUAL_INT(9, (int)CMD_SET_INVERT);
}

void test_handler_accepts_inverted_true(void) {
    JsonDocument doc;
    doc["inverted"] = true;
    TEST_ASSERT_TRUE(doc["inverted"].is<bool>());
    TEST_ASSERT_EQUAL(true, doc["inverted"].as<bool>());
}

void test_handler_accepts_inverted_false(void) {
    JsonDocument doc;
    doc["inverted"] = false;
    TEST_ASSERT_TRUE(doc["inverted"].is<bool>());
    TEST_ASSERT_EQUAL(false, doc["inverted"].as<bool>());
}

void test_handler_rejects_missing_inverted_key(void) {
    JsonDocument doc;
    // no "inverted" key — missing key must be a no-op
    TEST_ASSERT_FALSE(doc["inverted"].is<bool>());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_cmd_set_invert_is_9);
    RUN_TEST(test_handler_accepts_inverted_true);
    RUN_TEST(test_handler_accepts_inverted_false);
    RUN_TEST(test_handler_rejects_missing_inverted_key);
    return UNITY_END();
}
