#include "debug/debug_init.h"

#include "debug/can_debug.h"

#include "SimpleFOC.h"

void debug_init() {
  SimpleFOCDebug::enable(&CANDebug);
}

