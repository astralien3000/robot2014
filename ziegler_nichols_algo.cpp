
#include <device/output.hpp>
#include <device/input.hpp>

#include <filter/pid_filter.hpp>

//! \brief Represent a function by an array
//! \todo implement
class Function {
public:
  bool oscillating(void);
  int period(void);
  int& operator()(int t);
};

//! \brief Ziegler-Nichols method to configure a PidFilter
//! \param out : device controlled by a PidFilter
//! \param in : device used to see the result of a command
//! \param pid : the PidFilter to configure
void ziegler_nichols_algo(Output<s32>& out, Input<s32>& in, PidFilter& pid) {
  // Init
  s32 k = 0:
  pid.setGains(k,0,0);
  out.setValue(RESET_COMMAND);

  // Increase until oscillations
  Function data;
  while(!data.oscillating()) {

    // Increase PID
    k++;
    pid.setGain(k,0,0);

    // Test command
    out.setValue(TEST_COMMAND);

    // Measurement
    for(int i = 0 ; i < MAX_DATA ; i++) {
      data(i) = in.getValue();
      // SLEEP !! TODO
    }
    
    // Reset command
    out.setValue(RESET_COMMAND);
    // WAIT UNTIL OK !! TODO
  }

  // Configure PID
  int t = data.period();
  pid.setGain(0.60*k, 2*k/t, k*t/8);
}
