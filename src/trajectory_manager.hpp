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

  PidFilter& _pid_r;
  PidFilter& _pid_c;
  DiffFilter _diff_d;

  Vect<2, s32> _src;
  Vect<2, s32> _dst;

  struct CurvTrajectoryData {
    Vect<2, s32> _cen; // Center of the circle's curve
    s32 _dst_angle;
    s32 _ray;

    inline CurvTrajectoryData()
      : _cen(0,0), _dst_angle(0), _ray(0) {

    }
  };

  struct RectTrajectoryData {
    Vect<2, s32> _nor; // Normal of the Src->Dst segment

    inline RectTrajectoryData(void)
      : _nor(0,0) {

    }
  };

  union TrajectoryData {
    RectTrajectoryData rect;
    CurvTrajectoryData curv;

    inline TrajectoryData(void)
      : curv() {

    }
  };

  TrajectoryData _data;

  s32 _dist_cmd = 0;
  s32 _angle_cmd = 0;

  enum TrajectoryState {
    STOP,

    REACH_ANGLE_CURV,
    FOLLOW_TRAJECTORY_CURV,
    NEAR_END_CURV,

    REACH_ANGLE_RECT,
    FOLLOW_TRAJECTORY_RECT,
    NEAR_END_RECT
  };

  TrajectoryState _state;

public:
  //! \brief Constructor
  //! \param robot : A robot controller with dist/angle asserv
  //! \param pos : A device to get the x,y,angle position of the robot
  TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid_r, PidFilter& pid_c);
  
  //! \brief Set the new point to reach from current position, with a curve
  //! \param pos : Point to reach
  //! \param pseudo_ray : Distance between the center of the circle and the segment between Source and Destionation point
  //! \param way : true for going in the trigonometric way
  void gotoPosition(Vect<2, s32> pos, s32 pseudo_ray, bool way = true);

  //! \brief Set the new point to reach from current position
  void gotoPosition(Vect<2, s32> pos);
  
  //! \brief interrupt function
  void update(void);

  //! \brief Return if the robot reached destination
  bool isEnded(void);

  void reset(void);

private:
  void update_stop(void);

  void update_reach_angle(void);
  void update_follow_trajectory(void);
  void update_near_end(void);

  void update_reach_angle_rect(void);
  void update_follow_trajectory_rect(void);
  void update_near_end_rect(void);
};

#endif//TRAJECTORY_MANAGER_HPP
