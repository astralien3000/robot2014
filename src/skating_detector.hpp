#ifndef SKATING_DETECTOR_HPP
#define SKATING_DETECTOR_HPP

#include <device/input.hpp>

#include <base/integer.hpp>

//! \brief A virtual device which returns true if skating is detected.
/*!  
  
  Basically, it compares the ratio of the speed of each Input, and if
  there is a significant difference, getValue returns true.

  The third argument (limit) is supposed to be between 1?? and 100?? (to be measured).
  
*/
class SkatingDetector : public Input<bool> {
private:
  Input<s32>& _enc_int;
  Input<s32>& _enc_ext;
  s32 _limit;
  s32 _old_int;
  s32 _old_ext;
  s32 _last_ratio;

public:
  SkatingDetector(Input<s32>& enc_int, Input<s32>& enc_ext, s32 limit);

  bool getValue(void);

  s32 getLastRatio(void);
};

#endif//SKATING_DETECTOR_HPP
