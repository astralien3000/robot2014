#ifndef MY_RDS_HPP
#define MY_RDS_HPP

#include <device/eirbot2014/rds.hpp>

extern Rds rds;

void rds_init(void);
//returns true if at least one robot is detected.
bool check_for_collision(void);

#endif//MY_RDS_HPP
