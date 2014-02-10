#include <filter/pid_filter.hpp>
#include <filter/quadramp_filter.hpp>
#include <filter/diff_filter.hpp>
#include <filter/feedback_loop_filter.hpp>
#include <filter/composed_filter.hpp>

#include <device/stream/uart_stream.hpp>
#include <device/stream/eeprom_stream.hpp>

#include <math/safe_integer.hpp>

#include <device/eirbot2014/encoder.hpp>
#include <device/eirbot2014/motor.hpp>
//#include <device/eirbot2014/odometer.hpp>
#include <device/controller/motor_controller.hpp>
#include <device/controller/robot_controller.hpp>
#include <device/output_converter.hpp>
#include <device/input_converter.hpp>

#include <math/vect.hpp>
#include <math/matrix.hpp>

#include <hardware/uart.hpp>
#include <hardware/timer.hpp>
#include <hardware/xmem.hpp>
#include <hardware/eeprom.hpp>
#include <hardware/interrupts.hpp>

#define F_CPU 16000000l

//#include <util/delay.h>

#include "fpga.hpp"

// Filters
PidFilter id;

PidFilter pid_l;
PidFilter pid_r;
PidFilter pid_d;
PidFilter pid_a;

DiffFilter diff_l;
DiffFilter diff_r;
DiffFilter diff_d;
DiffFilter diff_a;

QuadrampFilter qramp_l;
QuadrampFilter qramp_r;
QuadrampFilter qramp_d;
QuadrampFilter qramp_a;

// Devices
Encoder<volatile u32> enc_l("left_encoder", &ENC_L);
Encoder<volatile u32> enc_r("right_encoder", &ENC_R);

InputConverter<s32, volatile u32> enc_conv_l(enc_l);
InputConverter<s32, volatile u32> enc_conv_r(enc_r);

Motor<volatile s8> mot_l("left_motor", &MOT_L);
Motor<volatile s8> mot_r("right_motor", &MOT_R);

OutputConverter<s32, volatile s8> mot_conv_l(mot_l);
OutputConverter<s32, volatile s8> mot_conv_r(mot_r);
  
//MotorController motc_l(mot_conv_l, enc_conv_l, qramp_l, diff_l, pid_l);
//MotorController motc_r(mot_conv_r, enc_conv_r, qramp_r, diff_r, pid_r);
MotorController motc_l(mot_conv_l, enc_conv_l, id, id, pid_l);

// TODO : REPAIRE
//Odometer odo(enc_l, enc_r);

//RobotController robot(motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);


void asserv_init(void) {
  id.setGains(1, 0, 0);

  // MotorControl
  diff_l.setDelta(4);
  pid_l.setGains(800, 0, 0);
  pid_l.setMaxIntegral(1000);
  pid_l.setOutShift(5);
  qramp_l.setFirstOrderLimit(100,100);
  qramp_l.setSecondOrderLimit(8,8);

  diff_r.setDelta(4);
  pid_r.setGains(800, 0, 0);
  pid_r.setMaxIntegral(1000);
  pid_r.setOutShift(11);
  qramp_r.setFirstOrderLimit(100,100);
  qramp_r.setSecondOrderLimit(8,8);

  // Odometer
  //odo.setImpPerCm(100);
  //odo.setDistEncoders(15);

  // Robot
  pid_a.setGains(1000, 10, 0);
  pid_a.setMaxIntegral(10000);
  pid_a.setOutShift(8);
  
  pid_d.setGains(100, 10, 0);
  pid_d.setMaxIntegral(10000);
  pid_d.setOutShift(7);

  qramp_a.setFirstOrderLimit(1,1);
  qramp_a.setSecondOrderLimit(1,1);

  qramp_d.setFirstOrderLimit(1,1);
  qramp_d.setSecondOrderLimit(1,1);
}

void fpga_init(void) {
  // Extenral memory initialization
  // Xmem::instance().init();
  // FPGA manual reste
  // DDRB |= (1<<0); 
  // PORTB &= ~(1<<0);
  // _delay_ms(500);
  // PORTB |= (1<<0);
  // PORTB &= ~(1<<0);
}

extern "C" void __cxa_pure_virtual() { while(1); }

//void ziegler_nichols_algo(Output<s32>&, Input<s32>&, PidFilter&);
void ziegler_nichols_algo(Output<s32>&, Input<volatile u32>&, PidFilter&);

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  Aversive::init();
  fpga_init();

  MOT_R = 0;
  MOT_L = 0;

  UartStream<0> io("stdio");
  EepromStream fs("eeprom");
  fs.setMode(Stream::BINARY);
  //Uart<0>::instance().init();

  //ziegler_nichols_algo(motc_l, enc_l, pid_l);
  //return 0;

  s32 dummy = 0;
  while(Aversive::isRunning()) {
    //Uart<0>::instance().send('d');
    //io << "time : " << (s16)FPGA_S << "s " << (s16)FPGA_MS << "ms " << (s16)FPGA_US << "us\n";
    //io << "position (x=" << (s16)POSX_FPGA << " ; y=" << (s16)POSY_FPGA << " ; a=" << (s16)ROT_FPGA << " )\n";
    dummy = 0;
    fs.seek(0);
    fs >> dummy;
    io << "read " << dummy << "\n";
    io >> dummy;
    fs.seek(0);
    fs << dummy;
    io << "write " << dummy << " " << (s32)55 << "\n";
    Aversive::sleep();
  }

  return 0;
}
