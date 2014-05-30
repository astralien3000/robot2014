#include "asserv.hpp"

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
//Encoder<volatile u32> enc_l("left_encoder", &ENC_L);
//Encoder<volatile u32> enc_r("right_encoder", &ENC_R);
decltype(enc_l) enc_l("left_encoder");
decltype(enc_r) enc_r("right_encoder");

//Encoder<volatile u32> enc_mot_l("left_motor_encoder", &ENC_MOT_L);
//Encoder<volatile u32> enc_mot_r("right_motor_encoder", &ENC_MOT_R);
decltype(enc_mot_l) enc_mot_l("left_motor_encoder");
decltype(enc_mot_r) enc_mot_r("right_motor_encoder");

// Motor<volatile s8> mot_l("left_motor", &MOT_L);
// Motor<volatile s8> mot_r("right_motor", &MOT_R);
decltype(mot_l) mot_l("left_motor");
decltype(mot_r) mot_r("right_motor");
  
MotorController motc_l(mot_l, enc_l, id, diff_l, pid_l);
MotorController motc_r(mot_r, enc_r, id, diff_r, pid_r);

Odometer odo(enc_l, enc_r);

RobotController _robot(motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);
SecureRobot robot(_robot, odo, skd_l, skd_r, mot_l, mot_r);
//SecureRobot robot(_robot, odo, skd_l, skd_r, motc_l, motc_r);

SkatingDetector skd_l(enc_mot_l, enc_l, 2);
SkatingDetector skd_r(enc_mot_r, enc_r, 2);

PositionManager pos(POSX_FPGA, POSY_FPGA, ROT_FPGA);

void asserv_init(void) {
  id.setGains(1, 0, 0);

  // MotorControl
  diff_l.setDelta(1);
  pid_l.setGains(200, 0, 0);
  pid_l.setMaxIntegral(0);
  pid_l.setOutShift(10);

  diff_r.setDelta(1);
  pid_r.setGains(200, 0, 0);
  pid_r.setMaxIntegral(0);
  pid_r.setOutShift(10);

  // Odometer
  odo.setDistanceMultiplicator(10);
  odo.setImpPerUnit(835);

  odo.setAngleMultiplicator(80);
  odo.setImpPerDeg(1327);

  // Motors
  mot_l.setMinCommand(-127);
  mot_l.setMaxCommand(127);
  mot_r.setMinCommand(-127);
  mot_r.setMaxCommand(127);

  // Position
  pos.setImpPerUnitX(82);
  pos.setImpPerUnitY(82);

  // Robot
  pid_a.setGains(480, 8, 0);
  pid_a.setMaxIntegral(6000);
  pid_a.setOutShift(6);
  
  pid_d.setGains(600, 8, 0);
  pid_d.setMaxIntegral(16000);
  // TEST PATINAGEss
  //pid_d.setGains(1600, 16, 0);
  //pid_d.setMaxIntegral(32000);
  pid_d.setOutShift(6);

  qramp_a.setFirstOrderLimit(30,30);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(10,10);
  qramp_d.setSecondOrderLimit(2,8);
}

void asserv_lockmode_passiv(void) {
  robot.setLockableMotors(mot_l, mot_r);
}

void asserv_lockmode_activ(void) {
  robot.setLockableMotors(motc_l, motc_r);
}

void asserv_speed_slow(void) {
  //  qramp_a.setFirstOrderLimit(15,15);
  qramp_a.setFirstOrderLimit(20,20);
  qramp_a.setSecondOrderLimit(2,2);

  //qramp_d.setFirstOrderLimit(5,5);
  //  qramp_d.setFirstOrderLimit(8,8);
  qramp_d.setFirstOrderLimit(10,10);
  qramp_d.setSecondOrderLimit(1,2);
}

void asserv_speed_normal(void) {
  qramp_a.setFirstOrderLimit(40,40);
  qramp_a.setSecondOrderLimit(4,4);

  //qramp_d.setFirstOrderLimit(13,13);
  qramp_d.setFirstOrderLimit(17,17);
  qramp_d.setSecondOrderLimit(2,2);
}

void asserv_speed_fast(void) {
  qramp_a.setFirstOrderLimit(40,40);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(22,22);
  //qramp_d.setFirstOrderLimit(20,20);
  qramp_d.setSecondOrderLimit(2,2);
}
