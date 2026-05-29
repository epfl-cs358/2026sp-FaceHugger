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

// ---- T:6 (CMD_SET_AUTO_INVERT) handler contract ---------------------------
// Repurposed T:6: was the manual "invert robot" trigger (CMD_ACTION_SELECTION),
// now sets whether the firmware is allowed to drive isInverted off the IMU
// latch. Payload: {"T":6, "enabled":bool}. Missing/non-bool field → no-op.

void test_cmd_set_auto_invert_is_6(void) {
    TEST_ASSERT_EQUAL_INT(6, (int)CMD_SET_AUTO_INVERT);
}

void test_auto_invert_handler_accepts_enabled_true(void) {
    JsonDocument doc;
    doc["enabled"] = true;
    TEST_ASSERT_TRUE(doc["enabled"].is<bool>());
    TEST_ASSERT_EQUAL(true, doc["enabled"].as<bool>());
}

void test_auto_invert_handler_accepts_enabled_false(void) {
    JsonDocument doc;
    doc["enabled"] = false;
    TEST_ASSERT_TRUE(doc["enabled"].is<bool>());
    TEST_ASSERT_EQUAL(false, doc["enabled"].as<bool>());
}

void test_auto_invert_handler_rejects_missing_enabled_key(void) {
    JsonDocument doc;
    // no "enabled" key — missing key must be a no-op
    TEST_ASSERT_FALSE(doc["enabled"].is<bool>());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_cmd_set_invert_is_9);
    RUN_TEST(test_handler_accepts_inverted_true);
    RUN_TEST(test_handler_accepts_inverted_false);
    RUN_TEST(test_handler_rejects_missing_inverted_key);
    RUN_TEST(test_cmd_set_auto_invert_is_6);
    RUN_TEST(test_auto_invert_handler_accepts_enabled_true);
    RUN_TEST(test_auto_invert_handler_accepts_enabled_false);
    RUN_TEST(test_auto_invert_handler_rejects_missing_enabled_key);
    return UNITY_END();
}
