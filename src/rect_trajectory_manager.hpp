#ifndef RECT_TRAJECTORY_MANAGER_HPP
#define RECT_TRAJECTORY_MANAGER_HPP

#include "trajectory_manager.hpp"

class RectTrajectoryManager : public TrajectoryManager {
public:
  //! \brief Constructor
  //! \param robot : The robot controller
  //! \param pos : The position sensor
  //! \param odo : The odometer sensor
  //! \param pid_r : The Filter for rect trajectory
  RectTrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid_r);

  //! \brief Set the new point to reach from current position
  void gotoPosition(Vect<2, s32> pos);

  //! \brief Set a distance forward or backward to reach
  /*!

    Whatever the mode is, if dist < 0, the robot will go backward, and
    forward if dist > 0

  */
  void gotoDistance(s32 dist);

  //! \brief Just reach angle (independent from mode)
  void gotoAngle(s32 angle);

private:
  //! \brief Filter used to correct error on normal vector
  PidFilter& _pid_r;

  DiffFilter _diff_a;

  //! \brief Normal vector of the Src->Dst segment
  Vect<2, s32> _nor;

  //! \brief Segment Src->Dst
  Vect<2, s32> _seg;

  //! \brief Distance between Src and Dst
  s32 _seg_len;

private:
  //! \brief First part of the trajectory : face the destination
  void update_reach_angle(void);

  //! \brief Main part of the trajectory
  void update_follow_trajectory(void);
  void update_follow_trajectory_dist(void);

  //! \brief What to do if robot is near the destination
  void update_near_end(void);

  static void _update_reach_angle(TrajectoryManager&);
  static void _update_follow_trajectory(TrajectoryManager&);
  static void _update_follow_trajectory_dist(TrajectoryManager&);
  static void _update_near_end(TrajectoryManager&);
};

#endif//RECT_TRAJECTORY_MANAGER_HPP
