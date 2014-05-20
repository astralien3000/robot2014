#include "devices.hpp"
#include "fpga.hpp"

FpgaServomotor<volatile u16, SERVO4_ADDR> basket_servo("basket_servo");

void servo_init(void) {
  basket_servo.setMinCommand(900);
  basket_servo.setMaxCommand(1650);
  basket_servo.setValue(BASKET_SERVO_UP_CMD);
}
