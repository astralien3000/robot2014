#include <base/array.hpp>

#include <device/output.hpp>
#include <device/input.hpp>
#include <device/stream/uart_stream.hpp>

#include <filter/pid_filter.hpp>

#include <hardware/timer.hpp>
#include <hardware/interrupts.hpp>


#define FPGA_MS   (*(volatile u16*)0x8082)
#define FPGA_S    (*(volatile u16*)0x8084)

const u32 RESET_COMMAND = 0;
const u32 TEST_COMMAND = 10000;
const s32 MAX_DATA = 200;

volatile bool finished = true;
volatile s32 compter = 0;

Output<s32>* _out;
//Input<s32>* _in;
Input<volatile u32>* _in;

//! \brief Represent a function by an array
//! \todo implement
class Function {
private:
  Array<MAX_DATA, s32> _data;
public:
  bool oscillating(void) {
    return false;
  }

  int period(void) {
    return 0;
  }

  s32& operator()(int t) {
    return _data[t];
  }
};

Function data;
s32 i = 0;

UartStream<0> io("stdio");

inline s32 time_ms(void) {
  return ((s32)FPGA_S << 16) + (s32)FPGA_MS;
}

//! \brief Ziegler-Nichols method to configure a PidFilter
//! \param out : device controlled by a PidFilter
//! \param in : device used to see the result of a command
//! \param pid : the PidFilter to configure
void ziegler_nichols_algo(Output<s32>& out, Input<volatile u32>& in, PidFilter& pid) {
  // Init
  s32 k = 1;
  pid.setGains(k,0,0);
  out.setValue(RESET_COMMAND);
  _out = &out;
  _in = &in;


  // Increase until oscillations
  while(!data.oscillating()) {
    io << "New Test with K = " << k << "\n";
    // Increase PID
    k *= 2;
    pid.setGains(k,0,0);

    finished = false;
    
    compter = 0;
    i = 0;

    io << "Goto " << (s32)TEST_COMMAND << "\n";
    s32 t1 = time_ms(), t2 = t1;
    while(!finished) {
      t2 = time_ms();
      if(t2 - t1 > 1) {
	if(compter % 3 == 0) {
	  // Test command
	  _out->setValue(TEST_COMMAND);
	}
      
	if(compter % 1 == 0) {
	  // Measurement
	  data(i++) = _in->getValue();
	  if(MAX_DATA <= i) {
	    finished = true;
	  }
	}

	io << (s32)TEST_COMMAND << " " << (s32)_in->getValue() << " " << (s32)pid.out() << "\n";
	  
	t1 = t2;
	//Uart<0>::instance().send('a');
	compter++;
      }
    }
    io << "Finished !\n";
    io << "Final state : " << (s32)in.getValue() << "\n";

    for(int i = 0 ; i < MAX_DATA ; i++) {
    }

    // Reset command
    io << "Returning to default state...\n";
    io << "Goto " << (s32)RESET_COMMAND << "\n";
    while(in.getValue() != RESET_COMMAND) {
      out.setValue(RESET_COMMAND);
    }
  }

  // Configure PID
  int t = data.period();
  pid.setGains(0.60*k, 2*k/t, k*t/8);
}
