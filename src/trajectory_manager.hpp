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
protected:
  Output< Vect<2, s32> >& _robot;
  Input< Vect<2, s32> >& _pos;
  Input< Vect<2, s32> >& _odo;

  enum TrajectoryState {
    STOP,
    REACH_ANGLE,
    FOLLOW_TRAJECTORY,
    NEAR_END,

    ////////
    MAX_STATES
  };

public:
  enum Mode {
    FORWARD,
    BACKWARD,
    FASTER
  };

public:
  //! \brief Constructor
  //! \param robot : A robot controller with dist/angle asserv
  //! \param odo : A device measuring distance and angle
  //! \param pos : A device to get the x,y,angle position of the robot
  TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos);
  
  void setMode(Mode m);

  //! \brief Return if the robot reached destination
  bool isEnded(void);

  //! \brief Reset command and stop the robot.
  void reset(void);

protected:
  Vect<2, s32> _src;
  Vect<2, s32> _dst;
  bool _backward;

  s32 _dist_cmd = 0;
  s32 _angle_cmd = 0;

  Mode _mod;

protected:
  TrajectoryState _state;
  Array<MAX_STATES, void (*)(TrajectoryManager&)> _state_handlers;
  Array<MAX_STATES, void (*)(TrajectoryManager&)> _state_change_handlers;

public:
  //! \brief interrupt function
  void update(void);

private:
  //! \brief Do not move !
  void update_stop(void);

  static void _update_stop(TrajectoryManager& t);
};

// TODO MOVE
#include "devices.hpp"


inline s32 nearest_cmd_angle(s32 angle, s32 cmd) {
  s32 res = (angle % 360);
  angle -= res;
  cmd = ((cmd + 180) % 360) - 180;

  if(180 < res - cmd) {
    angle += 360;
  }
  else if(res - cmd < -180) {
    angle -= 360;
  }

  return angle + cmd;
}

#warning "TODO : REMOVE THIS !!!"
#define REACH_ANGLE_RECT REACH_ANGLE
#define REACH_ANGLE_CURV REACH_ANGLE

#define NEAR_END_RECT NEAR_END
#define NEAR_END_CURV NEAR_END

#define FOLLOW_TRAJECTORY_RECT FOLLOW_TRAJECTORY
#define FOLLOW_TRAJECTORY_CURV FOLLOW_TRAJECTORY

#define update_reach_angle_rect update_reach_angle
#define update_reach_angle_curv update_reach_angle
#define update_follow_trajectory_rect update_follow_trajectory
#define update_follow_trajectory_curv update_follow_trajectory
#define update_near_end_rect update_near_end
#define update_near_end_curv update_near_end

#endif//TRAJECTORY_MANAGER_HPP
