#ifndef SECURE_ROBOT_HPP
#define SECURE_ROBOT_HPP

#include <base/integer.hpp>

#include <device/intput.hpp>
#include <device/output.hpp>

#include <math/vect.hpp>

//! \brief A Device which prevent the robot from moving while skating too long.
/*!
  
  In normal functionnment, the secure robot only forward commands to
  the robot controller, but if the robot is skating, it prevent to
  continue.

 */
class SecureRobot : public Output<Vect<2, s32>>, public Input<bool> {
private:
  Output< Vect<2, s32> >& _robot;
  Input<bool>& _skd_l, _skd_r;

public:
  SecureRobot(Output< Vect<2, s32> >& robot, Input<bool>& skd_l, Input<bool>& skd_r);

  bool getValue(void);

  void setValue(Vect<2, s32>);

  void unlock(void);
};

#endif//SECURE_ROBOT_HPP
