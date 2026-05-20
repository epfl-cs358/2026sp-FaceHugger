#include <unity.h>
#include <string.h>
#include <stdio.h>
#include "../../src/brain/csv_log.h"

void setUp(void) {}
void tearDown(void) {}

static int count_commas(const char* s) {
    int n = 0;
    for (; *s; ++s) if (*s == ',') ++n;
    return n;
}

void test_header_has_expected_column_count(void) {
    const char* h = csv_log::header();
    TEST_ASSERT_NOT_NULL(h);
    TEST_ASSERT_EQUAL_INT((int)csv_log::kColumnCount - 1, count_commas(h));
    // header ends with '\n'
    size_t L = strlen(h);
    TEST_ASSERT_TRUE(L > 0 && h[L - 1] == '\n');
}

void test_row_has_same_column_count_as_header(void) {
    csv_log::State s{};
    char buf[256];
    size_t n = csv_log::format_row(s, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(n < sizeof(buf));
    buf[n] = '\0';
    TEST_ASSERT_EQUAL_INT((int)csv_log::kColumnCount - 1, count_commas(buf));
    TEST_ASSERT_EQUAL_CHAR('\n', buf[n - 1]);
}

void test_row_formats_known_state(void) {
    csv_log::State s{};
    s.millis = 12345;
    s.robot_state = 1;
    s.gait = 2;
    s.is_moving = 1;
    s.is_inverted = 0;
    s.last_cmd_ms = 12000;
    s.target_x = 0.0f; s.target_y = 1.0f; s.target_yaw = 0.0f;
    s.active_x = 0.10f; s.active_y = 0.90f; s.active_yaw = 0.0f;
    for (int i = 0; i < 12; ++i) s.servo_angles[i] = 90.0f;
    char buf[256];
    size_t n = csv_log::format_row(s, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    buf[n] = '\0';
    // Spot-check the leading fields and one servo column.
    TEST_ASSERT_TRUE(strstr(buf, "12345,1,2,1,0,12000,") == buf);
    // Spot-check the motion float columns to catch ordering bugs.
    TEST_ASSERT_TRUE(strstr(buf, ",0.00,1.00,0.00,0.10,0.90,0.00,") != NULL);
    TEST_ASSERT_TRUE(strstr(buf, ",90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0\n") != NULL);
}

void test_format_row_returns_zero_on_overflow(void) {
    csv_log::State s{};
    char generous[256];
    size_t n = csv_log::format_row(s, generous, sizeof(generous));
    TEST_ASSERT_TRUE(n > 0);
    // Exactly n bytes is insufficient — snprintf needs room for the null terminator,
    // so the implementation must return 0.
    char tight[256];
    TEST_ASSERT_EQUAL_INT(0, (int)csv_log::format_row(s, tight, n));
    // One more byte is enough.
    TEST_ASSERT_TRUE(csv_log::format_row(s, tight, n + 1) > 0);
}

void test_ring_empty_view(void) {
    csv_log::Ring<32> r;
    const char* d1; size_t s1; const char* d2; size_t s2;
    r.view(d1, s1, d2, s2);
    TEST_ASSERT_EQUAL_INT(0, (int)s1);
    TEST_ASSERT_EQUAL_INT(0, (int)s2);
    TEST_ASSERT_EQUAL_INT(0, (int)r.size());
}

void test_ring_append_then_view(void) {
    csv_log::Ring<32> r;
    const char row[] = "abc\n";
    r.append(row, 4);
    const char* d1; size_t s1; const char* d2; size_t s2;
    r.view(d1, s1, d2, s2);
    TEST_ASSERT_EQUAL_INT(4, (int)s1);
    TEST_ASSERT_EQUAL_INT(0, (int)s2);
    TEST_ASSERT_EQUAL_MEMORY("abc\n", d1, 4);
}

void test_ring_evicts_oldest_row(void) {
    // Capacity 10, rows of 3 bytes each ("xx\n"). Three rows fill 9 bytes.
    // Appending a fourth (3 bytes) needs 12 > 10, so "aa\n" is evicted (oldest).
    // After eviction len_=6; 6+3=9 <= 10, so eviction stops — "bb\n" is kept.
    // Result: "bb\ncc\ndd\n" (9 bytes).
    csv_log::Ring<10> r;
    r.append("aa\n", 3);
    r.append("bb\n", 3);
    r.append("cc\n", 3);
    r.append("dd\n", 3);  // evicts "aa\n" only; "bb\n","cc\n","dd\n" remain
    TEST_ASSERT_EQUAL_INT(9, (int)r.size());
    const char* d1; size_t s1; const char* d2; size_t s2;
    r.view(d1, s1, d2, s2);
    char joined[16] = {0};
    if (s1) memcpy(joined, d1, s1);
    if (s2) memcpy(joined + s1, d2, s2);
    TEST_ASSERT_EQUAL_STRING("bb\ncc\ndd\n", joined);
}

void test_ring_handles_wrap(void) {
    csv_log::Ring<8> r;
    r.append("aaa\n", 4);
    r.append("bbb\n", 4);  // buf full: "aaa\nbbb\n"
    r.append("ccc\n", 4);  // evict "aaa\n", write "ccc\n" wrapping
    const char* d1; size_t s1; const char* d2; size_t s2;
    r.view(d1, s1, d2, s2);
    char joined[16] = {0};
    if (s1) memcpy(joined, d1, s1);
    if (s2) memcpy(joined + s1, d2, s2);
    TEST_ASSERT_EQUAL_STRING("bbb\nccc\n", joined);
}

void test_ring_clear(void) {
    csv_log::Ring<8> r;
    r.append("xx\n", 3);
    r.clear();
    TEST_ASSERT_EQUAL_INT(0, (int)r.size());
}

void test_ring_oversize_row_is_dropped(void) {
    csv_log::Ring<4> r;
    r.append("toolong\n", 8);
    TEST_ASSERT_EQUAL_INT(0, (int)r.size());
}

void test_ring_single_append_evicts_multiple_rows(void) {
    // Capacity 6. Two rows of 3 bytes leaves the ring exactly full.
    // Appending a 4-byte row must evict BOTH existing rows (outer while
    // iterates twice), since dropping just one row leaves only 3 bytes
    // of room — still < 4 needed.
    csv_log::Ring<6> r;
    r.append("aa\n", 3);
    r.append("bb\n", 3);
    TEST_ASSERT_EQUAL_INT(6, (int)r.size());
    r.append("ccc\n", 4);
    TEST_ASSERT_EQUAL_INT(4, (int)r.size());
    const char* d1; size_t s1; const char* d2; size_t s2;
    r.view(d1, s1, d2, s2);
    char joined[16] = {0};
    if (s1) memcpy(joined, d1, s1);
    if (s2) memcpy(joined + s1, d2, s2);
    TEST_ASSERT_EQUAL_STRING("ccc\n", joined);
}

void test_ring_accepts_row_equal_to_capacity(void) {
    csv_log::Ring<5> r;
    r.append("abcd\n", 5);
    TEST_ASSERT_EQUAL_INT(5, (int)r.size());
    const char* d1; size_t s1; const char* d2; size_t s2;
    r.view(d1, s1, d2, s2);
    TEST_ASSERT_EQUAL_INT(5, (int)s1);
    TEST_ASSERT_EQUAL_INT(0, (int)s2);
    TEST_ASSERT_EQUAL_MEMORY("abcd\n", d1, 5);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_header_has_expected_column_count);
    RUN_TEST(test_row_has_same_column_count_as_header);
    RUN_TEST(test_row_formats_known_state);
    RUN_TEST(test_format_row_returns_zero_on_overflow);
    RUN_TEST(test_ring_empty_view);
    RUN_TEST(test_ring_append_then_view);
    RUN_TEST(test_ring_evicts_oldest_row);
    RUN_TEST(test_ring_handles_wrap);
    RUN_TEST(test_ring_clear);
    RUN_TEST(test_ring_oversize_row_is_dropped);
    RUN_TEST(test_ring_single_append_evicts_multiple_rows);
    RUN_TEST(test_ring_accepts_row_equal_to_capacity);
    return UNITY_END();
}
