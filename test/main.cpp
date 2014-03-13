#include <iostream>
using namespace std;

//////

#include "../src/trajectory_manager.cpp"

/////////////////////////////////////////////////////

#include <cmath>

#include <filter/feedback_loop_filter.hpp>

class FakeRobot;

class FakePos : public Input< Vect<2, s32> > {
  friend class FakeRobot;

private:
  Vect<2, double> _pos;

public:
  FakePos() : _pos(0,0) {}

  Vect<2, s32> getValue(void) {
    return Vect<2, s32>(_pos.coord(0), _pos.coord(1));
  }
};

class FakeOdo : public Input< Vect<2, s32> > {
  friend class FakeRobot;
  friend int main(int, char**);

private:
  Vect<2, double> _da;

public:
  FakeOdo() : _da(0,0) {}

  Vect<2, s32> getValue(void) {
    return Vect<2, s32>(_da.coord(0), _da.coord(1));
  }
};


class FakeRobot : public Output< Vect<2, s32> > {
  friend int main(int, char**);

private:
  FakePos& _pos_man;
  FakeOdo& _odo;

  PidFilter _pid_d, _pid_a;
  QuadrampFilter _qramp_d, _qramp_a;

public:
  FakeRobot(FakePos& p, FakeOdo& o) : _pos_man(p), _odo(o) {
    _pid_d.setGains(10, 0, 0);
    _pid_d.setMaxIntegral(1000);
    _pid_d.setOutShift(5);

    _pid_a.setGains(10, 0, 0);
    _pid_a.setMaxIntegral(1000);
    _pid_a.setOutShift(5);

    _qramp_a.setFirstOrderLimit(1, 1);
    _qramp_a.setSecondOrderLimit(1, 1);

    _qramp_d.setFirstOrderLimit(1, 1);
    _qramp_d.setSecondOrderLimit(1, 1);
  }

  void setValue(Vect<2, s32> val) {
    Vect<2, double> prec = _odo._da;

    _odo._da.coord(0) += (double)(_pid_d.doFilter(_qramp_d.doFilter(val.coord(0)) - _odo._da.coord(0))) / 100;
    _odo._da.coord(1) += (double)(_pid_a.doFilter(_qramp_a.doFilter(val.coord(1)) - _odo._da.coord(1))) / 100;
    
    Vect<2, double> delta = _odo._da - prec;

    double dx = ((double)delta.coord(0)) * Math::cos<Math::DEGREE>(((double)delta.coord(1)));
    double dy = ((double)delta.coord(0)) * Math::sin<Math::DEGREE>(((double)delta.coord(1)));

    Vect<2, double> dp(dx, dy);

    _pos_man._pos += dp;
  }
};

int main(int argc, char* argv[]) {
  (void) argc;
  (void) argv;

  PidFilter pid_t;
  pid_t.setGains(13000, 500, 200);
  pid_t.setMaxIntegral(100000);
  pid_t.setOutShift(8);

  FakePos pos;
  FakeOdo odo;
  FakeRobot robot(pos, odo);
  TrajectoryManager traj(robot, odo, pos, pid_t);
  
  traj.gotoPosition(Vect<2, s32>(0, 1000), 0);

  for(int i = 0 ; i < 1600 ; i++) {
    traj.update();
    auto p = pos.getValue();

    cout << i << " " << p.coord(0) << " " << p.coord(1) << " " << odo.getValue().coord(0) << " " << odo.getValue().coord(1) << endl;
  }

  return 0;
}
