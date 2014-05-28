#include <system/scheduler.hpp>

#include "devices.hpp"

static u16 _secure_counter_s = 0;

static const u16 MATCH_DURATION_S = 30;

static void _secure_timer_tick(void) {
  if(_secure_counter_s < MATCH_DURATION_S) {
    _secure_counter_s++;
    io << _secure_counter_s;
  }
  else {
    robot.setUnlockable(false);
    robot.lock();
    io << _secure_counter_s;
    while(1);
  }

  _secure_counter_s++;
  while(1);
}

void secure_timer_init(void) {
  Task t(_secure_timer_tick);
  t.setPeriod(1000000);
  t.setRepeat();

  Scheduler::instance().addTask(t);
}
