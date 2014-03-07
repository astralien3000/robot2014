#include <iostream>
using namespace std;

//////

#include <base/pair.hpp>
#include <base/integer.hpp>
#include <math/vect.hpp>

#include <container/list.hpp>

#include <device/output.hpp>
#include <device/input.hpp>

#include <math/trigo.hpp>
#include <math/saturate.hpp>

#include <filter/pid_filter.hpp>
#include <filter/quadramp_filter.hpp>

class TrajectoryManager {
  friend int main(int, char**);

private:
  Output< Vect<2, s32> >& _robot;
  Input< Vect<2, s32> >& _pos;
  Input< Vect<2, s32> >& _odo;

  PidFilter& _pid;

  Vect<2, s32> _src;
  Vect<2, s32> _dst;
  s32 _pseudo_ray, _ray, _ddist;

  Vect<2, s32> _dir; // Direction Src->Dst
  Vect<2, s32> _mid; // Middle of the SctDst segment
  Vect<2, s32> _nor; // Normal
  Vect<2, s32> _cen; // Center of the circle's curve

  bool _beg, _end;
  s32 _err;
  Vect<2, s32> _cmd_end;

public:
  //! \brief Constructor
  //! \param robot : A robot controller with dist/angle asserv
  //! \param pos : A device to get the x,y,angle position of the robot
  TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid)
    : _robot(robot), _pos(pos), _odo(odo), _pid(pid) {
  }

  //! \brief Set the new point to reach from current position
  void gotoPosition(Vect<2, s32> pos, s32 pseudo_ray) {
    _src = _pos.getValue();
    _dst = pos;
    _pseudo_ray = pseudo_ray;

    _dir = _dst - _src;
    _mid = _src + _dir / 2;
    _nor = normal(_dir) / _dir.norm();

    _cen = _mid + _pseudo_ray * _nor;
    _ray = (_src - _cen).norm();

    _ddist = _dir.norm() / 8;

    _beg = true;
    _end = false;
    _err = 0;
  }

  void update(void) {
    
    Vect<2, s32> vray = _pos.getValue() - _cen;
    s32 dangle = ((_ddist * 180) / _ray) / Math::PI;
    s32 cmd_angle = Math::atan2<Math::DEGREE>(vray.coord(0), vray.coord(1)) + 90 + dangle;
    s32 cur_angle = _odo.getValue().coord(1);

    s32 rdiff = _ray - vray.norm();

    _err = rdiff;

    if(_end || (_pos.getValue() - _dst).norm() < 3) {
      if(!_end) {
	_cmd_end = _odo.getValue();
      }
      _end = true;
      _robot.setValue(_cmd_end);
      return;
    }

    if(_beg && Math::abs(cur_angle - cmd_angle) > 2) {
      _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0), cmd_angle));
    }
    else {
      _beg = false;
      _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + _ddist, cur_angle - _pid.doFilter(rdiff)));
    }
  }
};

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

    _odo._da.coord(0) += (double)(_pid_d.doFilter(_qramp_d.doFilter(val.coord(0)) - _odo._da.coord(0))) / 10;
    _odo._da.coord(1) += (double)(_pid_a.doFilter(_qramp_a.doFilter(val.coord(1)) - _odo._da.coord(1))) / 10;
    
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
  
  traj.gotoPosition(Vect<2, s32>(0, 100), 50);

  int _init = 200;
  for(int i = 0 ; i < _init ; i++) {
    robot.setValue(Vect<2, s32>(0, -270));
    //traj.update();
    auto p = pos.getValue();
    //cout << "d=" << robot._odo._da.coord(0) << " ; a=" << robot._odo._da.coord(1) << endl;
    //cout << "x=" << p.coord(0) << " ; y=" << p.coord(1) << endl;
    cout << i << " " << p.coord(0) << " " << p.coord(1) << " 0" << endl;
    //cout << endl;
  }

  for(int i = 0 ; i < 1600 ; i++) {
    robot.setValue(Vect<2, s32>(1000, 90));
    //traj.update();
    auto p = pos.getValue();
    //cout << "d=" << robot._odo._da.coord(0) << " ; a=" << robot._odo._da.coord(1) << endl;
    //cout << "x=" << p.coord(0) << " ; y=" << p.coord(1) << endl;
    cout << i+_init << " " << p.coord(0) << " " << p.coord(1) << " " << traj._err << " " << traj._ray << endl;
    //cout << endl;
  }

  return 0;
}
