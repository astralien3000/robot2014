#ifndef SECURE_ROBOT_HPP
#define SECURE_ROBOT_HPP

#include <base/integer.hpp>

#include <device/input.hpp>
#include <device/output.hpp>

#include <math/vect.hpp>

//! \brief A Device which prevents the robot from moving while skating
//! too long.
/*!
  
  In normal functionnment, the secure robot only forward commands to
  the robot controller, but if the robot is skating, it prevents to
  continue.

  (_state == true) <=> robot is skating
 */
class SecureRobot : public Output<Vect<2, s32>>, public Input<bool> {
private:
  Output< Vect<2, s32> >& _robot;
  Input< Vect<2, s32> >& _odo;

  Input<bool>& _skd_l;
  Input<bool>& _skd_r;
  Output<s32>* _mot_l;
  Output<s32>* _mot_r;

  bool _state;
  Vect<2, s32> _pos_block;
  s8 _last_update;
  s16 _sk_dur;
  bool _lockable;
  bool _unlockable;

public:
  SecureRobot(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input<bool>& skd_l, Input<bool>& skd_r, Output<s32>& mot_l, Output<s32>& mot_r);

  void update(void);

  bool getValue(void);

  void setValue(Vect<2, s32>);

  void unlock(void);
  void lock(void);

  void setLockable(bool);
  void setUnlockable(bool);

  void setLockableMotors(Output<s32>& mot_l, Output<s32>& mot_r);
};

#endif//SECURE_ROBOT_HPP
