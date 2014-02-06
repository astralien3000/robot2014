#include <filter/pid_filter.hpp>
#include <filter/quadramp_filter.hpp>
#include <filter/diff_filter.hpp>
#include <filter/feedback_loop_filter.hpp>
#include <filter/composed_filter.hpp>

#include <device/stream/uart_stream.hpp>
#include <math/safe_integer.hpp>

#include <device/eirbot2014/encoder.hpp>
#include <device/eirbot2014/motor.hpp>
#include <device/eirbot2014/odometer.hpp>
#include <device/eirbot2014/motor_controller.hpp>
#include <device/eirbot2014/robot_controller.hpp>

#include <math/vect.hpp>
#include <math/matrix.hpp>

#include <hardware/uart.hpp>
#include <hardware/timer.hpp>
#include <hardware/xmem.hpp>
#include <hardware/eeprom.hpp>
#include <hardware/interrupts.hpp>

#define F_CPU 16000000l

#include <util/delay.h>

#define ENC_R (*(volatile u32*)0x80A0)
#define ENC_L (*(volatile u32*)0x8098)
#define ENC_MOT_R (*(volatile u32*)0x8094)
#define ENC_MOT_L (*(volatile u32*)0x809C)

#define MOT_R (*(volatile s8*)0x8000)
#define MOT_L (*(volatile s8*)0x8001)
#define RESET_FPGA (*(volatile u8*)0x807F)

typedef Encoder<volatile u32> LeftEncoder;
typedef Encoder<volatile u32> RightEncoder;

typedef Motor<volatile s8> LeftMotor;
typedef Motor<volatile s8> RightMotor;

typedef MotorController<LeftMotor, LeftEncoder, QuadrampFilter, DiffFilter, PidFilter> LeftController;
typedef MotorController<RightMotor, RightEncoder, QuadrampFilter, DiffFilter, PidFilter> RightController;

typedef Odometer<LeftEncoder, RightEncoder> MyOdometer;

typedef RobotController<LeftController, RightController, MyOdometer, QuadrampFilter, PidFilter, PidFilter> MyRobot;


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
LeftEncoder enc_l("left_encoder", &ENC_L);
RightEncoder enc_r("left_encoder", &ENC_R);

LeftMotor mot_l("left_motor", &MOT_L);
RightMotor mot_r("left_motor", &MOT_R);
  
LeftController motc_l("left_controller", mot_l, enc_l, qramp_l, diff_l, pid_l);
RightController motc_r("right_controller", mot_r, enc_r, qramp_r, diff_r, pid_r);

MyOdometer odo("odometer", enc_l, enc_r);

MyRobot robot("robot_controller", motc_l, motc_r, odo, qramp_d, id, pid_d, qramp_a, id, pid_a);


void asserv_init(void) {
  id.setGains(1, 0, 0);

  // MotorControl
  diff_l.setDelta(4);
  pid_l.setGains(800, 0, 0);
  pid_l.setMaxIntegral(1000);
  pid_l.setOutShift(11);
  qramp_l.setFirstOrderLimit(100,100);
  qramp_l.setSecondOrderLimit(8,8);

  diff_r.setDelta(4);
  pid_r.setGains(800, 0, 0);
  pid_r.setMaxIntegral(1000);
  pid_r.setOutShift(11);
  qramp_r.setFirstOrderLimit(100,100);
  qramp_r.setSecondOrderLimit(8,8);

  // Odometer
  odo.setImpPerCm(100);
  odo.setDistEncoders(15);

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

u16 i = 0;

template<typename T>
void eeprom_write(u16 addr, T val) {
  for(u16 i = 0 ; i < sizeof(val) ; i++) {
    Eeprom::instance().write(addr+i, (u8)(val >> ((sizeof(val) - 1 - i) * 8)));
  }
}

template<typename T>
void eeprom_read(u16 addr, T& val) {
  u8 buff;
  for(u16 i = 0 ; i < sizeof(val) ; i++) {
    Eeprom::instance().read(addr+i, buff);
    val <<= 8;
    val += buff;
  }
}

void ihm_set_eeprom(UartStream<0>& io, const char* name, u16 addr) {
    s32 val = 0;
    eeprom_read(0, val);
    io << name << " = " << val << "\n";
    io << "Enter new " << name << " : ";
    io >> val;
    eeprom_write(0, val);
}

int main(int argc, char* argv[]) {
  Xmem::instance().init();
  //reset FPGA
  _delay_ms(300);
  DDRB |= (1<<0);
  _delay_ms(300);
  PORTB &= ~(1<<0);
  _delay_ms(300);
  _delay_ms(300);
  PORTB |= (1<<0);
  _delay_ms(300);
  PORTB &= ~(1<<0);
  _delay_ms(300);
  
  for(unsigned int i=0x8000;i<0x807F;i++) (*(volatile u8*)i) = 0;
  RESET_FPGA = 255;
  _delay_ms(300);
  RESET_FPGA =0;

  MOT_R = 0;
  MOT_L = 0;
  
  motc_l.inverse();
  motc_r.inverse();  

  asserv_init();

  Timer<0>& timer = Timer<0>::instance();
  timer.init();
  timer.setPrescaler<8>();
  Timer<0>::OverflowEvent& evt = timer.overflowEvent();
  evt.setFunction([]() {
      if(i++ % 30 == 0) {
	Vect<2, s32> cmd;
	cmd.coord(0) = 0;
	cmd.coord(1) = 10;
	robot.setValue(cmd);
      }
    });
  evt.start();

  //Interrupts::set();

  UartStream<0> io;

  while(1) {
    ihm_set_eeprom(io, "test", 0);
  }

  return 0;
}
