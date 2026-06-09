// Pure interpolatie tests: catmullRom + linearInterp
#include <unity.h>
#include <cstdint>

namespace playback {
    float catmullRom(float p0, float p1, float p2, float p3, float t) {
        float t2 = t * t, t3 = t2 * t;
        return 0.5f * (
            (2.0f * p1) +
            (-p0 + p2) * t +
            (2.0f*p0 - 5.0f*p1 + 4.0f*p2 - p3) * t2 +
            (-p0 + 3.0f*p1 - 3.0f*p2 + p3) * t3
        );
    }
    float linearInterp(long aSteps, uint32_t aTimeMs, long bSteps, uint32_t bTimeMs, uint32_t nowMs) {
        if (bTimeMs == aTimeMs) return (float)aSteps;
        float t = (float)((int64_t)nowMs - (int64_t)aTimeMs) /
                  (float)((int64_t)bTimeMs - (int64_t)aTimeMs);
        if (t < 0.0f) t = 0.0f; if (t > 1.0f) t = 1.0f;
        return (float)aSteps + ((float)bSteps - (float)aSteps) * t;
    }
}

using namespace playback;

void test_catmullrom_t0_returns_p1() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, catmullRom(0, 10, 20, 30, 0.0f));
}

void test_catmullrom_t1_returns_p2() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 20.0f, catmullRom(0, 10, 20, 30, 1.0f));
}

void test_catmullrom_midpoint_linear_for_constant_velocity() {
    // p0=0,p1=10,p2=20,p3=30 is exact linear → midpoint = 15
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 15.0f, catmullRom(0, 10, 20, 30, 0.5f));
}

void test_catmullrom_continuity_at_endpoints() {
    float v_at_1 = catmullRom(0, 10, 20, 30, 1.0f);
    float v_at_0_next = catmullRom(10, 20, 30, 40, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, v_at_1, v_at_0_next);
}

void test_linearinterp_at_start_returns_a() {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, linearInterp(10, 1000, 20, 2000, 1000));
}

void test_linearinterp_at_end_returns_b() {
    TEST_ASSERT_EQUAL_FLOAT(20.0f, linearInterp(10, 1000, 20, 2000, 2000));
}

void test_linearinterp_midpoint() {
    TEST_ASSERT_EQUAL_FLOAT(15.0f, linearInterp(10, 1000, 20, 2000, 1500));
}

void test_linearinterp_clamps_to_end() {
    TEST_ASSERT_EQUAL_FLOAT(20.0f, linearInterp(10, 1000, 20, 2000, 3000));
}

void test_linearinterp_clamps_to_start() {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, linearInterp(10, 1000, 20, 2000, 500));
}

void test_linearinterp_same_time_returns_a() {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, linearInterp(10, 1000, 20, 1000, 1000));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_catmullrom_t0_returns_p1);
    RUN_TEST(test_catmullrom_t1_returns_p2);
    RUN_TEST(test_catmullrom_midpoint_linear_for_constant_velocity);
    RUN_TEST(test_catmullrom_continuity_at_endpoints);
    RUN_TEST(test_linearinterp_at_start_returns_a);
    RUN_TEST(test_linearinterp_at_end_returns_b);
    RUN_TEST(test_linearinterp_midpoint);
    RUN_TEST(test_linearinterp_clamps_to_end);
    RUN_TEST(test_linearinterp_clamps_to_start);
    RUN_TEST(test_linearinterp_same_time_returns_a);
    return UNITY_END();
}
