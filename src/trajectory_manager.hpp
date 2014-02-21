#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>

#include <device/output.hpp>

class TrajectoryManager : public List< Vect<2, s32> > {
private:
  Output< Vect<2, s32> >& _robot;

public:
  TrajectoryManager(Output< Vect<2, s32> >& _robot);

  // TODO
};

#endif//TRAJECTORY_MANAGER_HPP
