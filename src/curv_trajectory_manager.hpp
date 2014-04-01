#include "rect_trajectory_manager.hpp"

class CurvTrajectoryManager : public RectTrajectoryManager {
public:
  //! \brief Constructor
  //! \param robot : The robot controller
  //! \param pos : The position sensor
  //! \param odo : The odometer sensor
  //! \param pid_r : The error filter for rect trajectory
  //! \param pid_c : The error filter for curved trajectory
  CurvTrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid_r, PidFilter& pid_c);

  //! \brief Set the new point to reach from current position, with a curve
  //! \param pos : Point to reach
  //! \param pseudo_ray : Distance between the center of the circle and the segment between Source and Destionation point
  //! \param way : true for going in the trigonometric way
  void gotoCurvPosition(Vect<2, s32> pos, s32 pseudo_ray, bool way = true);

private:
  //! \brief Filter used to correct the radius of the trajectory
  PidFilter& _pid_c;

  //! \brief Filter used to compute robot's speed
  DiffFilter _diff_d;

  //! \brief Center of the circle's curve
  Vect<2, s32> _cen;

  //! \brief Angle of the destination on the circle
  s32 _dst_angle;
  
  //! \brief Radius of the circle
  s32 _ray;

  //! \brief If the circle is going in the trigonometric way
  bool _way;

  //! \brief The angle in the case of (anti/)trigonometric way
  s32 _way_angle;

private:
  //! \brief First part of the trajectory : tangent to the circle
  void update_reach_angle(void);

  //! \brief Main part of the trajectory
  void update_follow_trajectory(void);

  //! \brief What to do if robot is near the destination
  void update_near_end(void);

  static void _update_reach_angle(TrajectoryManager&);
  static void _update_follow_trajectory(TrajectoryManager&);
  static void _update_near_end(TrajectoryManager&);
};
