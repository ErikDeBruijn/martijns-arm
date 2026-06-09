// Pure CSV-parse test voor motion_file::parseCsvLine — geen SD card nodig.
#include <unity.h>
#include <cstdlib>
#include <cstdint>

// Mirror van de pure parser (zonder Arduino/SD includes).
namespace motion_file {
struct Sample { uint32_t tMs; long steps[3]; };

bool parseCsvLine(const char* line, Sample& out) {
    if (!line || !*line) return false;
    if ((line[0] == 't' || line[0] == 'T') && (line[1] == ',' || line[1] == '_')) return false;
    char* p = const_cast<char*>(line); char* end;
    long t = std::strtol(p, &end, 10);
    if (end == p || *end != ',') return false;
    out.tMs = (uint32_t)t; p = end + 1;
    for (int i = 0; i < 3; i++) {
        long v = std::strtol(p, &end, 10);
        if (end == p) return false;
        out.steps[i] = v; p = end;
        if (i < 2) { if (*p != ',') return false; p++; }
    }
    return true;
}
}

using namespace motion_file;

void test_valid_line() {
    Sample s;
    TEST_ASSERT_TRUE(parseCsvLine("123,10,-20,30", s));
    TEST_ASSERT_EQUAL_UINT32(123, s.tMs);
    TEST_ASSERT_EQUAL_INT( 10, s.steps[0]);
    TEST_ASSERT_EQUAL_INT(-20, s.steps[1]);
    TEST_ASSERT_EQUAL_INT( 30, s.steps[2]);
}

void test_zero_line() {
    Sample s;
    TEST_ASSERT_TRUE(parseCsvLine("0,0,0,0", s));
    TEST_ASSERT_EQUAL_UINT32(0, s.tMs);
}

void test_large_values() {
    Sample s;
    TEST_ASSERT_TRUE(parseCsvLine("4294967295,1000000,-1000000,500000", s));
    TEST_ASSERT_EQUAL_INT(1000000, s.steps[0]);
}

void test_header_rejected() {
    Sample s;
    TEST_ASSERT_FALSE(parseCsvLine("t_ms,steps_1,steps_2,steps_3", s));
}

void test_empty_rejected() {
    Sample s;
    TEST_ASSERT_FALSE(parseCsvLine("", s));
}

void test_missing_field_rejected() {
    Sample s;
    TEST_ASSERT_FALSE(parseCsvLine("100,10,20", s));
}

void test_garbage_rejected() {
    Sample s;
    TEST_ASSERT_FALSE(parseCsvLine("abc,def,ghi,jkl", s));
}

void test_trailing_comma_extra_ignored() {
    // Extra data na 3e veld → strtol stopt en we accepteren — tolerantie OK.
    Sample s;
    TEST_ASSERT_TRUE(parseCsvLine("50,1,2,3,trailing", s));
    TEST_ASSERT_EQUAL_INT(3, s.steps[2]);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_valid_line);
    RUN_TEST(test_zero_line);
    RUN_TEST(test_large_values);
    RUN_TEST(test_header_rejected);
    RUN_TEST(test_empty_rejected);
    RUN_TEST(test_missing_field_rejected);
    RUN_TEST(test_garbage_rejected);
    RUN_TEST(test_trailing_comma_extra_ignored);
    return UNITY_END();
}
