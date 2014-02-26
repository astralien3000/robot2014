#include "asserv.hpp"

#include <device/output_converter.hpp>
#include <device/input_converter.hpp>

#include "devices.hpp"
#include "filters.hpp"
#include "fpga.hpp"

// To replace empty filters slots
static PidFilter id(1,0,0);

// Global filters
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

// Global devices
Encoder<volatile u32> enc_l("left_encoder", &ENC_L);
Encoder<volatile u32> enc_r("right_encoder", &ENC_R);

Motor<volatile s8> mot_l("left_motor", &MOT_L);
Motor<volatile s8> mot_r("right_motor", &MOT_R);

InputConverter<s32, volatile u32> enc_conv_l(enc_l);
InputConverter<s32, volatile u32> enc_conv_r(enc_r);

OutputConverter<s32, volatile s8> mot_conv_l(mot_l);
OutputConverter<s32, volatile s8> mot_conv_r(mot_r);
  
MotorController motc_l(mot_conv_l, enc_conv_l, id, diff_l, pid_l);
MotorController motc_r(mot_conv_r, enc_conv_r, id, diff_r, pid_r);

Odometer odo(enc_conv_l, enc_conv_r);

RobotController robot(motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);

void asserv_init(void) {
  id.setGains(1, 0, 0);

  // MotorControl
  diff_l.setDelta(4);
  pid_l.setGains(900, 0, 0);
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
  odo.setDistEncoders(3);

  // Robot
  pid_a.setGains(120, 1, 50);
  pid_a.setMaxIntegral(10000);
  pid_a.setOutShift(4);
  
  pid_d.setGains(70, 1, 20);
  pid_d.setMaxIntegral(1000);
  pid_d.setOutShift(4);

  qramp_a.setFirstOrderLimit(100,100);
  qramp_a.setSecondOrderLimit(10,10);

  qramp_d.setFirstOrderLimit(3,3);
  qramp_d.setSecondOrderLimit(1,1);
}
