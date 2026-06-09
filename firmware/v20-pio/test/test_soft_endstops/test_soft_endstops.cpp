#include <unity.h>
#include "../../src/soft_endstops.cpp"

using namespace soft_endstops;

void test_no_limits_passes_ref_through() {
    Limits L;
    auto r = clamp(50.0f, 100.0f, L);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.refDeg);
    TEST_ASSERT_FALSE(r.hitMin);
    TEST_ASSERT_FALSE(r.hitMax);
}

void test_within_range_passes_through() {
    Limits L{-90.0f, 90.0f};
    auto r = clamp(30.0f, 0.0f, L);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, r.refDeg);
    TEST_ASSERT_FALSE(r.hitMin);
    TEST_ASSERT_FALSE(r.hitMax);
}

void test_at_or_above_max_holds_position() {
    Limits L{-90.0f, 90.0f};
    auto r = clamp(95.0f, 91.0f, L);    // current al voorbij max
    TEST_ASSERT_EQUAL_FLOAT(91.0f, r.refDeg);
    TEST_ASSERT_TRUE(r.hitMax);
    TEST_ASSERT_FALSE(r.hitMin);
}

void test_at_or_below_min_holds_position() {
    Limits L{-90.0f, 90.0f};
    auto r = clamp(-95.0f, -91.0f, L);
    TEST_ASSERT_EQUAL_FLOAT(-91.0f, r.refDeg);
    TEST_ASSERT_TRUE(r.hitMin);
    TEST_ASSERT_FALSE(r.hitMax);
}

void test_only_min_set_max_irrelevant() {
    Limits L{-90.0f, NAN};
    auto r = clamp(200.0f, 150.0f, L);   // ref ver boven, geen max → pass
    TEST_ASSERT_EQUAL_FLOAT(200.0f, r.refDeg);
    TEST_ASSERT_FALSE(r.hitMax);
}

void test_only_max_set_min_irrelevant() {
    Limits L{NAN, 90.0f};
    auto r = clamp(-200.0f, -150.0f, L);
    TEST_ASSERT_EQUAL_FLOAT(-200.0f, r.refDeg);
    TEST_ASSERT_FALSE(r.hitMin);
}

void test_isSet() {
    TEST_ASSERT_FALSE(isSet(Limits{}));
    TEST_ASSERT_TRUE (isSet(Limits{0.0f, NAN}));
    TEST_ASSERT_TRUE (isSet(Limits{NAN, 0.0f}));
    TEST_ASSERT_TRUE (isSet(Limits{-1.0f, 1.0f}));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_no_limits_passes_ref_through);
    RUN_TEST(test_within_range_passes_through);
    RUN_TEST(test_at_or_above_max_holds_position);
    RUN_TEST(test_at_or_below_min_holds_position);
    RUN_TEST(test_only_min_set_max_irrelevant);
    RUN_TEST(test_only_max_set_min_irrelevant);
    RUN_TEST(test_isSet);
    return UNITY_END();
}
