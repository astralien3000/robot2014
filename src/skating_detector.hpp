#ifndef SKATING_DETECTOR_HPP
#define SKATING_DETECTOR_HPP

#include <device/input.hpp>

//! \brief A virtual device which returns true if skating is detected.
/*!  
  
  Basically, it compares the ratio of the speed of each Input, and if
  there is a significant difference, getValue returns true.

  The third argument (limit) is supposed to be between 1?? and 100?? (to be measured).
  
*/
class SkatingDetector : public Input<bool> {
private:
  Input<s32>& _encInt;
  Input<s32>& _encExt;
  int _limit;
  int _oldInt;
  int _oldExt;
  int _lastRatio;

public:
  SkatingDetector(Input<s32>&, Input<s32>&, int);

  bool getValue(void);
  int getLastRatio(void);
};

#endif//SKATING_DETECTOR_HPP
