#ifndef EIRBOT_SHELL_HPP
#define EIRBOT_SHELL_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>

extern Vect<2, s32> cmd;

void cmd_dist_angle(void);

void cmd_print_infos(void);
void cmd_print_pos(void);

void cmd_pid_set(void);
void cmd_odo_config(void);

#endif//EIRBOT_SHELL_HPP
