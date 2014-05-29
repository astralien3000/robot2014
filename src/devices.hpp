#ifndef DEVICES_HPP
#define DEVICES_HPP

//#include <device/eirbot2014/encoder.hpp>
#include <device/encoder/fpga_encoder.hpp>
//#include <device/eirbot2014/motor.hpp>
#include <device/motor/fpga_motor.hpp>
#include <device/eirbot2014/odometer.hpp>
#include <device/controller/motor_controller.hpp>
#include <device/controller/robot_controller.hpp>
//#include <device/eirbot2014/position_manager.hpp>

#include <device/stream/uart_stream.hpp>
#include <device/stream/eeprom_stream.hpp>
#include <device/stream/fpga_uart_stream.hpp>

#include "skating_detector.hpp"
#include "secure_robot.hpp"

#include "position_manager.hpp"

#include "fpga.hpp"

#include <device/servomotor/fpga_servomotor.hpp>

////////////////////////////////////////
// Physical devices

//// Servos
extern FpgaServomotor<volatile u16, SERVO4_ADDR> basket_servo;
#define BASKET_SERVO_UP_CMD 1650
#define BASKET_SERVO_DOWN_CMD 900

extern FpgaServomotor<volatile u16, SERVO3_ADDR> arba_servo;
#define ARBA_SERVO_UP_CMD 1650
#define ARBA_SERVO_DOWN_CMD 900

//// Incremental Encoders
//extern Encoder<volatile u32> enc_l;
//extern Encoder<volatile u32> enc_r;
extern FpgaEncoder<volatile u32, ENC_L_ADDR> enc_l;
extern FpgaEncoder<volatile u32, ENC_R_ADDR> enc_r;

//extern Encoder<volatile u32> enc_mot_l;
//extern Encoder<volatile u32> enc_mot_r;
extern FpgaEncoder<volatile u32, ENC_MOT_L_ADDR> enc_mot_l;
extern FpgaEncoder<volatile u32, ENC_MOT_R_ADDR> enc_mot_r;


//// Motors
//extern Motor<volatile s8> mot_l;
//extern Motor<volatile s8> mot_r;
extern FpgaMotor<volatile s8, MOT_L_ADDR> mot_l;
extern FpgaMotor<volatile s8, MOT_R_ADDR> mot_r;

//// Communication
extern FormattedStream& io;
//extern EepromStream file;
extern FpgaUartStream rds_stream;
extern FpgaUartStream ihm_io;

////////////////////////////////////////
// Virtual devices

// Motor controllers
extern MotorController motc_l;
extern MotorController motc_r;

// Position measurement
extern Odometer odo;
extern PositionManager pos;

// Robot's movement controller
extern SecureRobot robot;

// Skating Detectors
extern SkatingDetector skd_l;
extern SkatingDetector skd_r;

#endif//DEVICES_HPP
