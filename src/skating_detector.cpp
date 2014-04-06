#include "skating_detector.hpp"

#include <math/saturate.hpp>

SkatingDetector::SkatingDetector(Input<s32>& enc_int, Input<s32>& enc_ext, s32 limit)
  : _enc_int(enc_int), _enc_ext(enc_ext) {
  _limit = limit;
  _old_int = 0;
  _old_ext = 0;
  _last_ratio = 0;
}

bool SkatingDetector::getValue(void) {
  s32 new_int = _enc_int.getValue();
  s32 d_int = new_int - _old_int;

  s32 new_ext = _enc_ext.getValue();
  s32 d_ext = new_ext - _old_ext;

  _old_int = new_int;
  _old_ext = new_ext;

  if (d_int == 0) {
    // motor is not turning, no skating possible
    return false;
  }

  _last_ratio = d_ext / (d_int / 100);

  //! \warning assuming enc_ext is faster than enc_int !
  return Math::abs(d_int) >= 200 && Math::abs(_last_ratio) < _limit;
}

s32 SkatingDetector::getLastRatio(void) {
  return _last_ratio;
}
