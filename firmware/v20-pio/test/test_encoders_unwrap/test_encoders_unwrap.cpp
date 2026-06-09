// Pure logica test voor updateUnwrap — multi-turn delta tracking.
// We embedden de functie hier (zonder Arduino/Wire dependencies)
// om hem native te kunnen testen.
#include <unity.h>

namespace encoders {
    struct Channel {
        unsigned int muxChannel = 0;
        int          dir         = 1;
        float        unwrappedDeg = 0.0f;
        float        lastDeg      = 0.0f;
        bool         initialised  = false;
    };

    float updateUnwrap(Channel& ch, float newDegRaw) {
        if (!ch.initialised) {
            ch.lastDeg      = newDegRaw;
            ch.unwrappedDeg = 0.0f;
            ch.initialised  = true;
            return ch.unwrappedDeg;
        }
        float d = newDegRaw - ch.lastDeg;
        if (d >  180.0f) d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
        ch.unwrappedDeg += d;
        ch.lastDeg       = newDegRaw;
        return ch.unwrappedDeg;
    }
}

using namespace encoders;

void test_first_call_zeros_unwrap() {
    Channel ch;
    float u = updateUnwrap(ch, 250.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, u);
    TEST_ASSERT_TRUE(ch.initialised);
    TEST_ASSERT_EQUAL_FLOAT(250.0f, ch.lastDeg);
}

void test_small_positive_delta_accumulates() {
    Channel ch;
    updateUnwrap(ch, 100.0f);
    float u = updateUnwrap(ch, 110.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, u);
}

void test_small_negative_delta_accumulates() {
    Channel ch;
    updateUnwrap(ch, 100.0f);
    float u = updateUnwrap(ch, 80.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -20.0f, u);
}

void test_wrap_at_359_to_1_detects_forward() {
    Channel ch;
    updateUnwrap(ch, 358.0f);
    float u = updateUnwrap(ch, 1.0f);  // delta = -357 → wrap → +3
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.0f, u);
}

void test_wrap_at_1_to_359_detects_backward() {
    Channel ch;
    updateUnwrap(ch, 1.0f);
    float u = updateUnwrap(ch, 359.0f);  // delta = +358 → wrap → -2
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.0f, u);
}

void test_multi_rotation_forward() {
    Channel ch;
    updateUnwrap(ch, 0.0f);
    // 3 volledige rotaties via sequence van 90° stappen
    float u = 0.0f;
    for (int i = 1; i <= 12; i++) {
        float d = (float)((90 * i) % 360);
        u = updateUnwrap(ch, d);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3 * 360.0f, u);
}

void test_no_phantom_wrap_at_180_delta() {
    // Edge case: precies 180° delta. Implementatie: d>180 én d<-180. Bij exact 180 geen correctie.
    Channel ch;
    updateUnwrap(ch, 0.0f);
    float u = updateUnwrap(ch, 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 180.0f, u);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_first_call_zeros_unwrap);
    RUN_TEST(test_small_positive_delta_accumulates);
    RUN_TEST(test_small_negative_delta_accumulates);
    RUN_TEST(test_wrap_at_359_to_1_detects_forward);
    RUN_TEST(test_wrap_at_1_to_359_detects_backward);
    RUN_TEST(test_multi_rotation_forward);
    RUN_TEST(test_no_phantom_wrap_at_180_delta);
    return UNITY_END();
}
