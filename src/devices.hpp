#ifndef DEVICES_HPP
#define DEVICES_HPP

#include <device/eirbot2014/encoder.hpp>
#include <device/eirbot2014/motor.hpp>
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

////////////////////////////////////////
// Physical devices

//// Incremental Encoders
extern Encoder<volatile u32> enc_l;
extern Encoder<volatile u32> enc_r;

extern Encoder<volatile u32> enc_mot_l;
extern Encoder<volatile u32> enc_mot_r;

//// Motors
extern Motor<volatile s8> mot_l;
extern Motor<volatile s8> mot_r;

//// Communication
extern UartStream<0> io;
extern EepromStream file;
extern FpgaUartStream rds_stream;

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
