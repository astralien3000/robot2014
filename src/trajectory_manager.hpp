#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include <base/pair.hpp>
#include <base/integer.hpp>
#include <math/vect.hpp>

#include <container/list.hpp>

#include <device/output.hpp>
#include <device/input.hpp>

#include <math/trigo.hpp>
#include <math/saturate.hpp>

#include <filter/pid_filter.hpp>
#include <filter/diff_filter.hpp>
#include <filter/quadramp_filter.hpp>

class TrajectoryManager {
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

  DiffFilter _diff_d;

  bool _beg, _end;
  s32 _err;
  Vect<2, s32> _cmd_end;

  enum TrajectoryState {
    STOP,
    REACH_ANGLE,
    FOLLOW_TRAJECTORY,
    NEAR_END
  };

  TrajectoryState _state;

public:
  //! \brief Constructor
  //! \param robot : A robot controller with dist/angle asserv
  //! \param pos : A device to get the x,y,angle position of the robot
  TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid);
  
  //! \brief Set the new point to reach from current position
  void gotoPosition(Vect<2, s32> pos, s32 pseudo_ray);
  
  void update(void);

private:
  void update_stop(void);
  void update_reach_angle(void);
  void update_follow_trajectory(void);
  void update_near_end(void);
};

#endif//TRAJECTORY_MANAGER_HPP
