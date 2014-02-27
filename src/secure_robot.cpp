#include "secure_robot.hpp"


SecureRobot::SecureRobot(Output< Vect<2, s32> >& robot, Input<bool>& skd_l, Input<bool>& skd_r) {
  this->_robot = robot;
  this->_skd_l = skd_l;
  this->_skd_r = skd_r;
  this->_state = false;
  Task check([this](void) {
      if (this->_state)
	return; //do not update _state while unlock() is not called
      this->_state = this->_skd_l.getValue() || this->_skd_r.getValue();
      if (this->_state) {
	Vect<2, s32> zero(0, 0);
	this->setValue(zero);
      }
    });
  check.setRepeat();
  check.setPeriod(100000); //check 10 times per second if robot is skating
  Scheduler::instance().addTask(check);
}

bool SecureRobot::getValue(void) { //what is the best solution ?
  return this->_state;
  //return (this->_skd_l.getValue() && this->_skd_r.getValue());
}

void SecureRobot::setValue(Vect<2, s32> command) {
  if (this->_state) {
    //robot is skating, send "DO NOT MOVE"
    Vect<2, s32> zero(0, 0);
    this->_robot.setValue(zero);
  } else {
    this->_robot.setValue(command);
  }
}

void SecureRobot::unlock(void) {
  this->_state = false;
}
