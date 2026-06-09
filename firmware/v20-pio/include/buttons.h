#pragma once
#include <Arduino.h>

namespace buttons {

enum class Event { NONE, REC_SHORT, PLAY_SHORT, PLAY_LONG, DEL_LONG };

void init();
Event poll();   // call vanuit main loop; returnt eerstvolgende detected event of NONE

} // namespace buttons
