#include <unity.h>
#include "../../src/pid.cpp"

using namespace pid;

void test_zero_error_zero_output() {
    Gains g{1.0f, 0.1f, 0.5f}; Config c{}; State s{};
    TEST_ASSERT_EQUAL_FLOAT(0.0f, compute(0.0f, 0.01f, g, c, s));
}

void test_p_only_proportional() {
    Gains g{2.0f, 0.0f, 0.0f}; Config c{}; State s{};
    TEST_ASSERT_EQUAL_FLOAT(20.0f, compute(10.0f, 0.01f, g, c, s));
}

void test_deadband_suppresses_output_and_holds_integral() {
    Gains g{10.0f, 1.0f, 0.0f}; Config c{}; c.deadbandErr = 0.5f; State s{};
    TEST_ASSERT_EQUAL_FLOAT(0.0f, compute(0.3f, 0.1f, g, c, s));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.iAccum);
}

void test_integral_accumulates() {
    Gains g{0.0f, 1.0f, 0.0f}; Config c{}; State s{};
    compute(1.0f, 0.1f, g, c, s);  // i = 0.1
    compute(1.0f, 0.1f, g, c, s);  // i = 0.2
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.iAccum);
}

void test_integral_clamp_caps_growth() {
    Gains g{0.0f, 1.0f, 0.0f}; Config c{}; c.iClampAbs = 0.5f; State s{};
    for (int i = 0; i < 100; i++) compute(1.0f, 0.1f, g, c, s);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, s.iAccum);
}

void test_output_saturation() {
    Gains g{100.0f, 0.0f, 0.0f}; Config c{}; c.outputMax = 50.0f; State s{};
    TEST_ASSERT_EQUAL_FLOAT(50.0f, compute(10.0f, 0.01f, g, c, s));
}

void test_derivative_acts_on_change() {
    Gains g{0.0f, 0.0f, 1.0f}; Config c{}; State s{};
    compute(0.0f, 0.1f, g, c, s);                  // baseline, init
    float out = compute(1.0f, 0.1f, g, c, s);      // delta-err = 1, dt=0.1 → d=10
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, out);
}

void test_zero_dt_returns_zero_no_state_change() {
    Gains g{1.0f, 1.0f, 1.0f}; Config c{}; State s{}; s.iAccum = 0.42f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, compute(5.0f, 0.0f, g, c, s));
    TEST_ASSERT_EQUAL_FLOAT(0.42f, s.iAccum);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_zero_output);
    RUN_TEST(test_p_only_proportional);
    RUN_TEST(test_deadband_suppresses_output_and_holds_integral);
    RUN_TEST(test_integral_accumulates);
    RUN_TEST(test_integral_clamp_caps_growth);
    RUN_TEST(test_output_saturation);
    RUN_TEST(test_derivative_acts_on_change);
    RUN_TEST(test_zero_dt_returns_zero_no_state_change);
    return UNITY_END();
}
