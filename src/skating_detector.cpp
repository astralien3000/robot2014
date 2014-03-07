#include "skating_detector.hpp"

SkatingDetector::SkatingDetector(Input<s32>& encInt, Input<s32>& encExt, int limit) : _encInt(encInt), _encExt(encExt) {
  this->_limit = limit;
  this->_oldInt = 0;
  this->_oldExt = 0;
  this->_lastRatio = 0;
}

bool SkatingDetector::getValue(void) {
  int newInt = this->_encInt.getValue();
  int DInt = newInt - this->_oldInt;
  if (DInt == 0) {
    return false;
  }
  int newExt = this->_encExt.getValue();
  int DExt = newExt - this->_oldExt;
  this->_oldInt = newInt;
  this->_oldExt = nexExt;
  this->_lastRatio = DExt / DInt;
  if (this->_lastRatio < 0) {
    return true;
  }
  return (this->_lastRatio < this->_limit); //assuming encExt is faster than encInt !
}

int SkatingDetector::getLastRatio(void) {
  return this->_lastRatio;
}
