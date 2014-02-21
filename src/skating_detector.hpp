#ifndef SKATING_DETECTOR_HPP
#define SKATING_DETECTOR_HPP

#include <device/input.hpp>

//! \brief A virtual device which returns true if skating is detected.
/*!  
  
  Basically, it compares the ratio of the speed of each Input, and if
  there is a significant difference, getValue returns true.
  
*/
class SkatingDetector : public Input<bool> {
private:
  Input<s32>& _enc1;
  Input<s32>& _enc2;

public:
  SkatingDetector(Input<s32>&, Input<s32>&);

  bool getValue(void);
};

#endif//SKATING_DETECTOR_HPP
