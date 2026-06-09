#include <unity.h>

namespace recording {
    float lowpass(float prev, float now, float alpha) {
        return alpha * now + (1.0f - alpha) * prev;
    }
}

using namespace recording;

void test_alpha_one_passes_through() {
    TEST_ASSERT_EQUAL_FLOAT(50.0f, lowpass(10.0f, 50.0f, 1.0f));
}

void test_alpha_zero_holds_prev() {
    TEST_ASSERT_EQUAL_FLOAT(10.0f, lowpass(10.0f, 50.0f, 0.0f));
}

void test_alpha_half_averages() {
    TEST_ASSERT_EQUAL_FLOAT(30.0f, lowpass(10.0f, 50.0f, 0.5f));
}

void test_step_response_converges() {
    float v = 0.0f;
    for (int i = 0; i < 100; i++) v = lowpass(v, 100.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, v);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_alpha_one_passes_through);
    RUN_TEST(test_alpha_zero_holds_prev);
    RUN_TEST(test_alpha_half_averages);
    RUN_TEST(test_step_response_converges);
    return UNITY_END();
}
