#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <filter/pid_filter.hpp>
#include <filter/diff_filter.hpp>
#include <filter/quadramp_filter.hpp>

////////////////////////////////////////
// Filters about robot's movement

extern PidFilter pid_l;
extern PidFilter pid_r;
extern PidFilter pid_d;
extern PidFilter pid_a;

extern DiffFilter diff_l;
extern DiffFilter diff_r;
extern DiffFilter diff_d;
extern DiffFilter diff_a;

extern QuadrampFilter qramp_l;
extern QuadrampFilter qramp_r;
extern QuadrampFilter qramp_d;
extern QuadrampFilter qramp_a;

#endif//FILTERS_HPP
