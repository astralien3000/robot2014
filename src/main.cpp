// #include <filter/pid_filter.hpp>
// #include <filter/diff_filter.hpp>
// #include <filter/quadramp_filter.hpp>
// #include <filter/feedback_loop_filter.hpp>
// #include <filter/composed_filter.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"

// #include <device/stream/uart_stream.hpp>
// #include <device/stream/eeprom_stream.hpp>

// #include <math/safe_integer.hpp>

// #include <device/eirbot2014/encoder.hpp>
// #include <device/eirbot2014/motor.hpp>
// #include <device/eirbot2014/odometer.hpp>
// #include <device/controller/motor_controller.hpp>
// #include <device/controller/robot_controller.hpp>
// #include <device/output_converter.hpp>
// #include <device/input_converter.hpp>
// #include <device/eirbot2014/position_manager.hpp>

#include "devices.hpp"

// #include <math/vect.hpp>
// #include <math/matrix.hpp>

// #include <hardware/uart.hpp>
// #include <hardware/timer.hpp>
// #include <hardware/xmem.hpp>
// #include <hardware/eeprom.hpp>
#include <hardware/interrupts.hpp>

#include <system/scheduler.hpp>
#include "fpga.hpp"

// Filters
// PidFilter id;

// PidFilter pid_l;
// PidFilter pid_r;
// PidFilter pid_d;
// PidFilter pid_a;

// DiffFilter diff_l;
// DiffFilter diff_r;
// DiffFilter diff_d;
// DiffFilter diff_a;

// QuadrampFilter qramp_l;
// QuadrampFilter qramp_r;
// QuadrampFilter qramp_d;
// QuadrampFilter qramp_a;

// Devices
// Encoder<volatile u32> enc_l("left_encoder", &ENC_L);
// Encoder<volatile u32> enc_r("right_encoder", &ENC_R);

// InputConverter<s32, volatile u32> enc_conv_l(enc_l);
// InputConverter<s32, volatile u32> enc_conv_r(enc_r);

// Motor<volatile s8> mot_l("left_motor", &MOT_L);
// Motor<volatile s8> mot_r("right_motor", &MOT_R);

// OutputConverter<s32, volatile s8> mot_conv_l(mot_l);
// OutputConverter<s32, volatile s8> mot_conv_r(mot_r);
  
// MotorController motc_l(mot_conv_l, enc_conv_l, PidFilter::identity(), diff_l, pid_l);
// MotorController motc_r(mot_conv_r, enc_conv_r, PidFilter::identity(), diff_r, pid_r);
// //MotorController motc_l(mot_conv_l, enc_conv_l, PidFilter::identity(), PidFilter::identity(), pid_l);

// Odometer odo(enc_conv_l, enc_conv_r);

// //PositionManager pos(odo);

// RobotController robot(motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);

// IHM

// void asserv_init(void) {
//   id.setGains(1, 0, 0);

//   // MotorControl
//   diff_l.setDelta(4);
//   pid_l.setGains(1000, 0, 0);
//   pid_l.setMaxIntegral(1000);
//   pid_l.setOutShift(11);
//   //qramp_l.setFirstOrderLimit(80,80);
//   //qramp_l.setSecondOrderLimit(2,2);

//   diff_r.setDelta(4);
//   pid_r.setGains(1000, 0, 0);
//   pid_r.setMaxIntegral(1000);
//   pid_r.setOutShift(11);
//   //qramp_r.setFirstOrderLimit(80,80);
//   //qramp_r.setSecondOrderLimit(2,2);

//   // Odometer
//   odo.setImpPerCm(100);
//   odo.setDistEncoders(15);

//   // Robot
//   pid_a.setGains(600, 1, 0);
//   pid_a.setMaxIntegral(10000);
//   pid_a.setOutShift(4);
  
//   pid_d.setGains(200, 0, 0);
//   pid_d.setMaxIntegral(1000);
//   pid_d.setOutShift(5);

//   qramp_a.setFirstOrderLimit(100,100);
//   qramp_a.setSecondOrderLimit(3,3);

//   qramp_d.setFirstOrderLimit(100,100);
//   qramp_d.setSecondOrderLimit(1,1);
// }


Scheduler& sched = Scheduler::instance();

void control_init(void) {
#if defined (__AVR_ATmega128__)
  Task t([](void) {
      robot.setValue(cmd);
      //pos.update();
    });

  t.setPeriod(8000);
  t.setRepeat();
  sched.addTask(t);

  Interrupts::set();
#endif
}

extern "C" void __cxa_pure_virtual() { while(1); }


int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  Aversive::init();
  asserv_init();
  fpga_init();

  file.setMode(Stream::BINARY);
  
  MOT_R = 0;
  MOT_L = 0;

  cmd.coord(0) = 0;
  cmd.coord(1) = 0;

  control_init();

  RELATION = 15100;

  motc_l.inverse();
  //motc_r.inverse();
  enc_r.inverse();
  
  //motc_l.setValue(50);
  //motc_r.setValue(50);
  
  //cmd.coord(0) = 200;
  //cmd.coord(1) = 90;
  
  while(Aversive::isRunning()) {
    cmd_print_infos();
    //cmd_print_pos();
    //cmd_pid_set();
    cmd_dist_angle();
  }

  return 0;
}
