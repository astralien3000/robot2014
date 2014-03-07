#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include <base/pair.hpp>
#include <base/integer.hpp>
#include <math/vect.hpp>

#include <container/list.hpp>

#include <device/output.hpp>

class TrajectoryManager : public List< Vect<2, s32> > {
private:
  Output< Vect<2, s32> >& _robot;
  Input< Vect<2, s32> >& _pos;

  Vect<2, s32> _src;
  Vect<2, s32> _dst;
  s32 _curve;

public:
  //! \brief Choose if the robot must go forward or can go backward if it is faster
  enum TrajectoryMode {
    MODE_ANGLE_2PI,
    MODE_ANGLE_PI
  };

public:
  //! \brief Constructor
  //! \param robot : A robot controller with dist/angle asserv
  //! \param pos : A device to get the x,y,angle position of the robot
  TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& pos);

  //! \brief Set the new point to reach from current position
  void gotoPosition(Vect<2, s32> pos, s32 curve);

  //! \brief Stop the trajectory in order to restart it later.
  void pause(void);

  //! \brief Resume the paused trajectory
  void resume(void);
};

#endif//TRAJECTORY_MANAGER_HPP
