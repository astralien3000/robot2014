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

Encoder<volatile u32> enc_mot_l("left_motor_encoder", &ENC_MOT_L);
Encoder<volatile u32> enc_mot_r("right_motor_encoder", &ENC_MOT_R);

Motor<volatile s8> mot_l("left_motor", &MOT_L);
Motor<volatile s8> mot_r("right_motor", &MOT_R);
  
MotorController motc_l(mot_l, enc_l, id, diff_l, pid_l);
MotorController motc_r(mot_r, enc_r, id, diff_r, pid_r);

Odometer odo(enc_l, enc_r);

RobotController _robot(motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);
SecureRobot robot(_robot, odo, skd_l, skd_r, mot_l, mot_r);

SkatingDetector skd_l(enc_mot_l, enc_l, 2);
SkatingDetector skd_r(enc_mot_r, enc_r, 2);

PositionManager pos(POSX_FPGA, POSY_FPGA, ROT_FPGA);

void asserv_init(void) {
  id.setGains(1, 0, 0);

  // MotorControl
  diff_l.setDelta(1);
  pid_l.setGains(200, 0, 0);
  pid_l.setMaxIntegral(1000);
  pid_l.setOutShift(10);

  diff_r.setDelta(1);
  pid_r.setGains(200, 0, 0);
  pid_r.setMaxIntegral(1000);
  pid_r.setOutShift(10);

  // Odometer
  odo.setImpPerUnit(81);
  odo.setImpPerDeg(28);

  // Position
  pos.setImpPerUnitX(82);
  pos.setImpPerUnitY(82);

  // Robot
  pid_a.setGains(480, 16, 0);
  pid_a.setMaxIntegral(800);
  pid_a.setOutShift(7);
  
  pid_d.setGains(70, 1, 20);
  pid_d.setMaxIntegral(500);
  pid_d.setOutShift(4);

  qramp_a.setFirstOrderLimit(30,30);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(10,10);
  qramp_d.setSecondOrderLimit(2,2);
}
