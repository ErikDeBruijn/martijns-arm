// Native unit test voor led::isOnAt — pure logica, geen hardware.
// Build: pio test -e native -f test_led
#include <unity.h>

// Voorkom dat we de hele FastLED+Arduino chain trekken — test alleen de pure logica.
// Repliceer de enum en functie hier (mirror van include/led.h).
namespace led {
    enum Anim { OFF, SOLID, BLINK_1HZ, BLINK_5HZ, PULSE };
    bool isOnAt(Anim anim, uint32_t nowMs) {
        switch (anim) {
            case OFF:       return false;
            case SOLID:     return true;
            case BLINK_1HZ: return (nowMs % 1000) < 500;
            case BLINK_5HZ: return (nowMs %  200) < 100;
            case PULSE:     return (nowMs %  800) < 400;
        }
        return false;
    }
}

void test_off_is_always_off() {
    TEST_ASSERT_FALSE(led::isOnAt(led::OFF, 0));
    TEST_ASSERT_FALSE(led::isOnAt(led::OFF, 1234));
    TEST_ASSERT_FALSE(led::isOnAt(led::OFF, 99999));
}

void test_solid_is_always_on() {
    TEST_ASSERT_TRUE(led::isOnAt(led::SOLID, 0));
    TEST_ASSERT_TRUE(led::isOnAt(led::SOLID, 1234));
}

void test_blink_1hz_50pct_duty() {
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_1HZ,   0));
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_1HZ, 250));
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_1HZ, 499));
    TEST_ASSERT_FALSE(led::isOnAt(led::BLINK_1HZ, 500));
    TEST_ASSERT_FALSE(led::isOnAt(led::BLINK_1HZ, 999));
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_1HZ,1000));  // wraps
}

void test_blink_5hz_50pct_duty() {
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_5HZ,  0));
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_5HZ, 99));
    TEST_ASSERT_FALSE(led::isOnAt(led::BLINK_5HZ,100));
    TEST_ASSERT_FALSE(led::isOnAt(led::BLINK_5HZ,199));
    TEST_ASSERT_TRUE (led::isOnAt(led::BLINK_5HZ,200));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_off_is_always_off);
    RUN_TEST(test_solid_is_always_on);
    RUN_TEST(test_blink_1hz_50pct_duty);
    RUN_TEST(test_blink_5hz_50pct_duty);
    return UNITY_END();
}
