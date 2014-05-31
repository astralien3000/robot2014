#include "secure_timer.hpp"

#include <system/scheduler.hpp>

#include "devices.hpp"

static u16 _secure_counter_tick = 0;
static bool _go = false;



static const u16 MATCH_DURATION_TICK = 2700;
//static const u16 MATCH_DURATION_TICK = 1000;

void secure_timer_tick(void) {
  if(_go) {
    _secure_counter_tick++;
    if (secure_timer_time() > 92) {
      arba_servo.setValue(ARBA_SERVO_UNLOCK_CMD);
    }      

    if(secure_timer_time() < (87)) {
      //io << secure_timer_time() << "\n";
    }
    else {
      robot.setUnlockable(false);
      robot.lock();
      io << secure_timer_time() << " END\n";
      //while(1);
    }
  }
  //_secure_counter_tick++;
  //while(1);
}

//Task t(_secure_timer_tick);

void secure_timer_init(void) {
  _go = true;
  // _secure_counter_tick = 0;

  // t.setPeriod(1000000);
  // t.setRepeat();

  // Scheduler::instance().addTask(t);
}



u16 secure_timer_time(void) {
  return (u16)((u32)_secure_counter_tick * (u32)90 / (u32)MATCH_DURATION_TICK);
}
