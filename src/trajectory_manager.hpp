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

public:
  TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& pos);

  
};

#endif//TRAJECTORY_MANAGER_HPP
