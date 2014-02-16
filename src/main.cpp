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
#include <device/eirbot2014/odometer.hpp>
#include <device/controller/motor_controller.hpp>
#include <device/controller/robot_controller.hpp>
#include <device/output_converter.hpp>
#include <device/input_converter.hpp>
#include <device/eirbot2014/position_manager.hpp>

#include <math/vect.hpp>
#include <math/matrix.hpp>

#include <hardware/uart.hpp>
#include <hardware/timer.hpp>
#include <hardware/xmem.hpp>
#include <hardware/eeprom.hpp>
#include <hardware/interrupts.hpp>

#include <system/scheduler.hpp>

#define F_CPU 16000000l

#if defined (__AVR_ATmega128__)
#include <util/delay.h>
#endif

#include <string.h>

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
  
MotorController motc_l(mot_conv_l, enc_conv_l, PidFilter::identity(), diff_l, pid_l);
MotorController motc_r(mot_conv_r, enc_conv_r, PidFilter::identity(), diff_r, pid_r);
//MotorController motc_l(mot_conv_l, enc_conv_l, PidFilter::identity(), PidFilter::identity(), pid_l);

Odometer odo(enc_conv_l, enc_conv_r);
PositionManager pos(odo);

RobotController robot(motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);

// IHM
UartStream<0> io("stdio");
EepromStream f("eeprom");

void asserv_init(void) {
  id.setGains(1, 0, 0);

  // MotorControl
  diff_l.setDelta(4);
  pid_l.setGains(1000, 0, 0);
  pid_l.setMaxIntegral(1000);
  pid_l.setOutShift(11);
  //qramp_l.setFirstOrderLimit(80,80);
  //qramp_l.setSecondOrderLimit(2,2);

  diff_r.setDelta(4);
  pid_r.setGains(1000, 0, 0);
  pid_r.setMaxIntegral(1000);
  pid_r.setOutShift(11);
  //qramp_r.setFirstOrderLimit(80,80);
  //qramp_r.setSecondOrderLimit(2,2);

  // Odometer
  odo.setImpPerCm(100);
  odo.setDistEncoders(15);

  // Robot
  pid_a.setGains(600, 1, 0);
  pid_a.setMaxIntegral(10000);
  pid_a.setOutShift(4);
  
  pid_d.setGains(200, 0, 0);
  pid_d.setMaxIntegral(1000);
  pid_d.setOutShift(5);

  qramp_a.setFirstOrderLimit(100,100);
  qramp_a.setSecondOrderLimit(3,3);

  qramp_d.setFirstOrderLimit(100,100);
  qramp_d.setSecondOrderLimit(1,1);
}

void fpga_init(void) {
#if defined (__AVR_ATmega128__)
  // External memory initialization
  Xmem::instance().init();
  // FPGA manual reste
  DDRB |= (1<<0); 
  PORTB &= ~(1<<0);
  _delay_ms(500);
  PORTB |= (1<<0);
  _delay_ms(1);
  PORTB &= ~(1<<0);
#endif
}

Vect<2, s32> cmd;

Scheduler& sched = Scheduler::instance();

void control_init(void) {
#if defined (__AVR_ATmega128__)
  /*
  Timer<0>& timer = Timer<0>::instance();
  timer.init();
  timer.setPrescaler<64>();
  Timer<0>::OverflowEvent& evt = timer.overflowEvent();
  evt.setFunction([]() {
      robot.setValue(cmd);
      //Uart<0>::instance().send('b');
    });
  evt.start();
  */
  Task t([](void) {
      robot.setValue(cmd);
      pos.update();
    });

  t.setPeriod(8000);
  t.setRepeat();
  sched.addTask(t);

  Interrupts::set();
#endif
}

extern "C" void __cxa_pure_virtual() { while(1); }

void ziegler_nichols_algo(Output<s32>&, Input<s32>&, PidFilter&);
//void ziegler_nichols_algo(Output<s32>&, Input<volatile u32>&, PidFilter&);

#include <device/output.hpp>

class FakeDistanceInput : public Input<s32> {
private:
  Odometer& _odo;
  
public:
  FakeDistanceInput(Odometer& odo) : _odo(odo) {}

  s32 getValue(void) {
    return _odo.getValue().coord(1);
  }
};

class FakeDistanceOutput : public Output<s32> {
private:
  RobotController& _bot;
  
public:
  FakeDistanceOutput(RobotController& bot) : _bot(bot) {}

  void setValue(s32 val) {
    Vect<2, s32> cmd;
    cmd.coord(0) = 0;
    cmd.coord(1) = val;
    _bot.setValue(cmd);
  }
};

void cmd_dist_angle(void) {
  mot_l.setValue(0);
  mot_r.setValue(0);
  
  s32 aux = 0;
  io << "Waiting for command...\n";
  io << "distance+=";
  io >> aux;
  cmd.coord(1) += aux;
  io << "angle+=";
  io >> aux;
  cmd.coord(0) += aux;
  
  io << "GOTO " << cmd.coord(1) << " " << cmd.coord(0) << "\n";
}

void cmd_print_infos(void) {
  io << "time : " << (s16)FPGA_S << "s ";
  io << (s16)FPGA_MS << "ms " ;
  io << (s16)FPGA_US << "us\n";

  io << "position (x=" << ((s32)POSX_FPGA >> 16);
  io << " ; y=" << ((s32)POSY_FPGA >> 16);
  io << " ; a1=" << ((ROT_FPGA >> 8) & 0xFF);
  io << " a2=" << (ROT_FPGA & 0xFF) << " )\n";

  io << "encoder (l=" << (u32)ENC_L;
  io << " ; r=" << (u32)ENC_R << ")\n";
  
  io << "distance=" << (s32)odo.getValue().coord(1);
  io << " ; angle=" << (s32)odo.getValue().coord(0) << "\n";
}

void cmd_print_pos(void) {
  io << "position (x=" << (s32)pos.x() << " ; y=" << (s32)pos.y() << " ; a=" << (s32)pos.angle() << " )\n";
  io << "encoder (l=" << (u32)ENC_L << " ; r=" << (u32)ENC_R << ")\n";
  io << "distance=" << (s32)odo.getValue().coord(1) << " ; angle=" << (s32)odo.getValue().coord(0) << "\n";
}

void pid_set_gains(PidFilter& pid, u16 off) {
  s32 p = 0, i = 0, d = 0;

  f.seek(off);
  f >> p;
  f >> i;
  f >> d;

  io << "P(" << p << ")=";
  io >> p;
  io << "I(" << i << ")=";
  io >> i;
  io << "D(" << d << ")=";
  io >> d;
  pid.setGains(p,i,d);

  f.seek(off);
  f << p << i << d;
}

void cmd_pid_set(void) {
  char str[32];
  io << "Choose filter :\n";
  io >> str;
  if(strcmp("left", str) == 0) {
    pid_set_gains(pid_l, 0);
  }
  else if(strcmp("right", str) == 0) {
    pid_set_gains(pid_r, 3*sizeof(s32));
  }
  else if(strcmp("dist", str) == 0) {
    pid_set_gains(pid_a, 6*sizeof(s32));
  }
  else if(strcmp("angle", str) == 0) {
    pid_set_gains(pid_d, 9*sizeof(s32));
  }
  else {
    io << "error! " << 404 << "\n";
  }
}

void cmd_odo_config(void) {
  s32 ipc = 0, de = 0;
  io << "ImpPerCm : ";
  io >> ipc;
  
  io << "DistEncoders : ";
  io >> de;

  odo.setImpPerCm(ipc);
  odo.setDistEncoders(de);
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  Aversive::init();
  asserv_init();
  fpga_init();

  f.setMode(Stream::BINARY);
  
  MOT_R = 0;
  MOT_L = 0;

  cmd.coord(0) = 0;
  cmd.coord(1) = 0;

  control_init();

  RELATION = 15100;

  motc_l.inverse();
  //motc_r.inverse();
  enc_r.inverse();
  
  FakeDistanceInput fo(odo);
  FakeDistanceOutput fb(robot);

  s32 dummy = 0;

  while(Aversive::isRunning()) {
    cmd_print_infos();
    //cmd_print_pos();
    //cmd_pid_set();
    cmd_dist_angle();
  }

  return 0;
}
