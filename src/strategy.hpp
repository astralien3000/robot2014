#ifndef STRATEGY_HPP
#define STRATEGY_HPP

enum State {
  BEGIN,
  SEARCH_ACTION,
  REACH_ACTION,
  DO_ACTION,
  SKATED,
  MAX_STATE
};

void do_your_job(void);

#endif //STRATEGY_HPP
