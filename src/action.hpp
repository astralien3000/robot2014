#ifndef ACTION_HPP
#define ACTION_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>

#include "secure_robot.hpp"
#include "rect_trajectory_manager.hpp"
#include "position_manager.hpp"
#include "rds.hpp"

enum Side {
  RED,
  YELLOW,
  MAX_SIDE
};

enum Error {
  SUCCESS,
  IMPOSSIBLE,
  SKATING,
  MAX_ERROR
};

class Action {
protected:
  u8 _static_priority = 10;

public:

  //! \brief Help the strategy to choose which action to do first
  /*!
    
    The action with the highest priority will be chosen. The priority can be dependent of distance, time, etc...
    The priority should be 0 or less if it has been done.
    
   */
  //! \todo Add the const attribute to it?
  //! \todo Should not it be abstract?
  virtual s16 priority(void);

  //! \brief Gives a point to reach to do the action
  /*!

    The doAction function will be called just after the point has been reached.

   */
  //! \todo Add the const attribute to it?
  //! \todo Should not it be abstract?
  virtual Vect<2, s32> controlPoint(void);

  //! \brief Do the action
  /*!

    The action must check if there is no collisions.

   */
  //! \todo Change the return type to an enum or an integer to specify an error code if one occured.
  //! \todo Should not it be abstract?
  virtual enum Error doAction(void);

  inline void done(void) {
    _static_priority = 0;
  }

  inline void resetPriority(void) {
    _static_priority = 1;
  }

public:
  
  static PositionManager& positionManager(void);
  static RectTrajectoryManager& trajectoryManager(void);
  static SecureRobot& robot(void);

  static void setPositionManager(PositionManager&);
  static void setTrajectoryManager(RectTrajectoryManager&);
  static void setRobot(SecureRobot&);

  static Side side;

protected:
  static u8 _fruit;
  
private:
  //! \brief Position manager common to all actions
  static PositionManager* _pos;

  //! \brief Trajectory manager common to all actions
  static RectTrajectoryManager* _traj;

  //! \brief Robot Secure common to all actions
  static SecureRobot* _robot;
};

#endif//ACTION_HPP
