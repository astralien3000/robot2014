#include "eirbot_shell.hpp"

#include "devices.hpp"
#include "filters.hpp"
#include "fpga.hpp"

#include <string.h>


UartStream<0> io("stdio");
EepromStream file("eeprom");

Vect<2, s32> cmd;

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
  //io << "position (x=" << (s32)pos.x() << " ; y=" << (s32)pos.y() << " ; a=" << (s32)pos.angle() << " )\n";
  io << "encoder (l=" << (u32)ENC_L << " ; r=" << (u32)ENC_R << ")\n";
  io << "distance=" << (s32)odo.getValue().coord(1) << " ; angle=" << (s32)odo.getValue().coord(0) << "\n";
}

void pid_set_gains(PidFilter& pid, u16 off) {
  s32 p = 0, i = 0, d = 0;

  file.seek(off);
  file >> p;
  file >> i;
  file >> d;

  io << "P(" << p << ")=";
  io >> p;
  io << "I(" << i << ")=";
  io >> i;
  io << "D(" << d << ")=";
  io >> d;
  pid.setGains(p,i,d);

  file.seek(off);
  file << p << i << d;
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
