#ifndef DEVICES_HPP
#define DEVICES_HPP

#include <device/eirbot2014/encoder.hpp>
#include <device/eirbot2014/motor.hpp>
#include <device/eirbot2014/odometer.hpp>
#include <device/controller/motor_controller.hpp>
#include <device/controller/robot_controller.hpp>
#include <device/eirbot2014/position_manager.hpp>

////////////////////////////////////////
// Physical devices

//// Incremental Encoders
extern Encoder<volatile u32> enc_l;
extern Encoder<volatile u32> enc_r;

//// Motors
extern Motor<volatile s8> mot_l;
extern Motor<volatile s8> mot_r;

//// Communication
extern UartStream<0> io;
extern EepromStream file;

////////////////////////////////////////
// Virtual devices

// Motor controllers
extern MotorController motc_l;
extern MotorController motc_r;

// Position measurement
extern Odometer odo;
extern PositionManager pos;

// Robot's movement controller
extern RobotController robot;


#endif//DEVICES_HPP
