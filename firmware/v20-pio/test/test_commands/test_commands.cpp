#include <unity.h>
#include "../../src/commands.cpp"

using namespace commands;

void test_empty_invalid() {
    auto pc = parse("");
    TEST_ASSERT_FALSE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::UNKNOWN, pc.verb);
}

void test_unknown_verb_invalid() {
    auto pc = parse("BOGUS");
    TEST_ASSERT_FALSE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::UNKNOWN, pc.verb);
}

void test_strips_leading_arrow() {
    auto pc = parse(">HELP");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::HELP, pc.verb);
}

void test_case_insensitive() {
    auto pc = parse("status");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::STATUS, pc.verb);
}

void test_simple_verbs() {
    TEST_ASSERT_EQUAL(Verb::HELP,      parse("HELP").verb);
    TEST_ASSERT_EQUAL(Verb::STATUS,    parse("STATUS").verb);
    TEST_ASSERT_EQUAL(Verb::HOME,      parse("HOME").verb);
    TEST_ASSERT_EQUAL(Verb::DEL,       parse("DEL").verb);
    TEST_ASSERT_EQUAL(Verb::LIMITS,    parse("LIMITS").verb);
    TEST_ASSERT_EQUAL(Verb::LIMITSAVE, parse("LIMITSAVE").verb);
    TEST_ASSERT_EQUAL(Verb::ENCRAW,    parse("ENCRAW").verb);
    TEST_ASSERT_EQUAL(Verb::TMCSTATUS, parse("TMCSTATUS").verb);
}

void test_mode_idle() {
    auto pc = parse("MODE IDLE");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::MODE, pc.verb);
    TEST_ASSERT_EQUAL(Mode::IDLE, pc.mode);
}

void test_mode_playback() {
    auto pc = parse(">MODE PLAYBACK");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(Mode::PLAYBACK, pc.mode);
}

void test_mode_unknown_invalid() {
    auto pc = parse("MODE FOO");
    TEST_ASSERT_FALSE(pc.valid);
}

void test_mode_missing_arg_invalid() {
    auto pc = parse("MODE");
    TEST_ASSERT_FALSE(pc.valid);
}

void test_tune_kp() {
    auto pc = parse("TUNE KP 1 1.4");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::TUNE, pc.verb);
    TEST_ASSERT_EQUAL(GainKey::KP, pc.gainKey);
    TEST_ASSERT_EQUAL_INT(1, pc.jointIdx);
    TEST_ASSERT_EQUAL_FLOAT(1.4f, pc.value);
}

void test_tune_negative_value() {
    auto pc = parse("TUNE KI 0 -0.5");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(GainKey::KI, pc.gainKey);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, pc.value);
}

void test_tune_bad_idx_invalid() {
    auto pc = parse("TUNE KP 3 1.0");
    TEST_ASSERT_FALSE(pc.valid);
}

void test_tune_bad_float_invalid() {
    auto pc = parse("TUNE KP 1 abc");
    TEST_ASSERT_FALSE(pc.valid);
}

void test_current_in_range() {
    auto pc = parse("CURRENT 2 60");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL_INT(2, pc.jointIdx);
    TEST_ASSERT_EQUAL_INT(60, pc.percent);
}

void test_current_out_of_range_invalid() {
    TEST_ASSERT_FALSE(parse("CURRENT 0 101").valid);
    TEST_ASSERT_FALSE(parse("CURRENT 0 -5").valid);
}

void test_limitset_min() {
    auto pc = parse("LIMITSET 1 MIN");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL_INT(1, pc.jointIdx);
    TEST_ASSERT_EQUAL(LimitSide::MIN, pc.limitSide);
}

void test_limitset_max() {
    auto pc = parse("LIMITSET 0 MAX");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(LimitSide::MAX, pc.limitSide);
}

void test_limitset_bad_side_invalid() {
    TEST_ASSERT_FALSE(parse("LIMITSET 0 FOO").valid);
}

void test_limitclr_valid() {
    auto pc = parse("LIMITCLR 1");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL_INT(1, pc.jointIdx);
}

void test_limitclr_bad_idx_invalid() {
    TEST_ASSERT_FALSE(parse("LIMITCLR -1").valid);
    TEST_ASSERT_FALSE(parse("LIMITCLR").valid);
}

void test_whitespace_robustness() {
    auto pc = parse("   >   TUNE   KP   1   2.5   ");
    TEST_ASSERT_TRUE(pc.valid);
    TEST_ASSERT_EQUAL(Verb::TUNE, pc.verb);
    TEST_ASSERT_EQUAL_FLOAT(2.5f, pc.value);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_empty_invalid);
    RUN_TEST(test_unknown_verb_invalid);
    RUN_TEST(test_strips_leading_arrow);
    RUN_TEST(test_case_insensitive);
    RUN_TEST(test_simple_verbs);
    RUN_TEST(test_mode_idle);
    RUN_TEST(test_mode_playback);
    RUN_TEST(test_mode_unknown_invalid);
    RUN_TEST(test_mode_missing_arg_invalid);
    RUN_TEST(test_tune_kp);
    RUN_TEST(test_tune_negative_value);
    RUN_TEST(test_tune_bad_idx_invalid);
    RUN_TEST(test_tune_bad_float_invalid);
    RUN_TEST(test_current_in_range);
    RUN_TEST(test_current_out_of_range_invalid);
    RUN_TEST(test_limitset_min);
    RUN_TEST(test_limitset_max);
    RUN_TEST(test_limitset_bad_side_invalid);
    RUN_TEST(test_limitclr_valid);
    RUN_TEST(test_limitclr_bad_idx_invalid);
    RUN_TEST(test_whitespace_robustness);
    return UNITY_END();
}
