#include "trajectory.hpp"
#include "devices.hpp"
#include "filters.hpp"

#include <system/scheduler.hpp>

PidFilter pid_rt;
PidFilter pid_ct;
CurvTrajectoryManager traj(robot, odo, pos, pid_rt, pid_ct);

static Scheduler& sched = Scheduler::instance();

void control_init(void) {
  Task t([](void) {
      traj.update();
    });

  t.setPeriod(64000);
  t.setRepeat();
  sched.addTask(t);

  pid_ct.setGains(1000, 50, 100);
  pid_ct.setMaxIntegral(25600);
  pid_ct.setOutShift(10);

  pid_rt.setGains(5, 0, 0);
  pid_rt.setMaxIntegral(100000);
  pid_rt.setOutShift(14);
}

void trajectory_reset(void) {
  pid_l.reset();
  pid_r.reset();
  pid_d.reset();
  pid_a.reset();

  qramp_d.reset(odo.getValue().coord(0));
  qramp_a.reset(odo.getValue().coord(1));

  pid_ct.reset();
  pid_rt.reset();

  traj.reset();
}
