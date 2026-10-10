#pragma once

#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

namespace config {
    // Drivetrain
    inline const double dt_track_width = 13.5;
    inline const double dt_wheel_diameter = lemlib::Omniwheel::NEW_275;

    inline int dt_rpm = 600;
    inline uint8_t dt_horizontal_drift = 2;

    inline const std::initializer_list<int8_t> right_motor_ports = {-1, -12, };
    inline const std::initializer_list<int8_t> left_motor_ports = {8, 15};

    
    inline const int8_t intake_motor_port = 11; // Intake
    inline const std::initializer_list<int8_t> cascade_motor_ports = {5, 6}; // Cascade
    inline const std::initializer_list<int8_t> claw_motor_ports = {11, 13}; // Claw
    inline const int8_t arm_motor_port = 14; // Arm

    //inline const char tongue_ADI = 'B';
    //inline const char wing_ADI = 'F';
    //inline const char middle_goal_top_ADI = 'H';
    //inline const char middle_goal_bottom_ADI = 'A';

    // Odom
    inline const int8_t imu_port = 20;
    inline const int8_t vertical_encoder_1_port = 20;
    inline const double vertical_wheel_1_diameter = lemlib::Omniwheel::NEW_2;
    inline const double vertical_wheel_1_distance = 0;
    inline const int8_t vertical_encoder_2_port = 20;
    inline const double vertical_wheel_2_diameter = lemlib::Omniwheel::NEW_2;
    inline const double vertical_wheel_2_distance = 0;
    inline const int8_t horizontal_encoder_port = 20;
    inline const double horizontal_wheel_diameter = lemlib::Omniwheel::NEW_2;
    inline const double horizontal_wheel_distance = 0;

    // Lateral PID
    inline const double lateral_kP = 10;
    inline const double lateral_kI = 0;
    inline const double lateral_kD = 3;

    inline const double lateral_anti_windup = 3;
    inline const double lateral_small_error_range = 1;
    inline const double lateral_small_error_range_timeout = 100;
    inline const double lateral_large_error_range = 3;
    inline const double lateral_large_error_range_timeout = 500;
    inline const double lateral_slew = 20;

     // Angular PID
    inline const double angular_kP = 2;
    inline const double angular_kI = 0;
    inline const double angular_kD = 10;

    inline const double angular_anti_windup = 3;
    inline const double angular_small_error_range = 1;
    inline const double angular_small_error_range_timeout = 000;
    inline const double angular_large_error_range = 3;
    inline const double angular_large_error_range_timeout = 500;
    inline const double angular_slew = 0;

} // namespace config

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
inline pros::MotorGroup leftMotorGroup(config::left_motor_ports, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotorGroup(config::right_motor_ports, pros::MotorGearset::blue);
inline pros::Motor intake(config::intake_motor_port, pros::MotorGearset::blue);
inline pros::MotorGroup cascade(config::cascade_motor_ports, pros::MotorGearset::blue);
inline pros::MotorGroup claw(config::claw_motor_ports, pros::MotorGearset::blue);
inline pros::Motor arm(config::arm_motor_port, pros::MotorGearset::blue, pros::v5::MotorUnits::degrees);

// Drive train settings
inline lemlib::Drivetrain driveTrain(&leftMotorGroup, &rightMotorGroup, config::dt_track_width, config::dt_wheel_diameter, config::dt_rpm, config::dt_horizontal_drift);

// Pneumatics
//inline pros::adi::Pneumatics tongueMech(config::tongue_ADI, false);
//inline pros::adi::Pneumatics wing(config::wing_ADI, false);

// Imu
inline pros::Imu imu(config::imu_port);

// Tracking Wheels
inline pros::Rotation verticalEncoder1(config::vertical_encoder_1_port);
inline lemlib::TrackingWheel vertical1(&verticalEncoder1, config::vertical_wheel_1_diameter, config::vertical_wheel_1_distance);
inline pros::Rotation verticalEncoder2(config::vertical_encoder_2_port);
inline lemlib::TrackingWheel vertical2(&verticalEncoder2, config::vertical_wheel_2_diameter, config::vertical_wheel_2_distance);
inline pros::Rotation horizontalEncoder(config::horizontal_encoder_port);
inline lemlib::TrackingWheel horizontal(&horizontalEncoder, config::horizontal_wheel_diameter, config::horizontal_wheel_distance);

// Sensors
inline lemlib::OdomSensors odom(&vertical1, &vertical2, &horizontal, nullptr, &imu);

// PID
inline lemlib::ControllerSettings lateral_controller(config::lateral_kP, config::lateral_kI, config::lateral_kD, config::lateral_anti_windup, config::lateral_small_error_range, config::lateral_small_error_range_timeout, config::lateral_large_error_range, config::lateral_large_error_range_timeout, config::lateral_slew);
inline lemlib::ControllerSettings angular_controller(config::angular_kP, config::angular_kI, config::angular_kD, config::angular_anti_windup, config::angular_small_error_range, config::angular_small_error_range_timeout, config::angular_large_error_range, config::angular_large_error_range_timeout, config::angular_slew);

// Drive curves
inline lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband (out of 127)
                                            10, // minimum output for movement (out of 127)
                                            1.019 // curve gain (a - value)
);
inline lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband (out of 127)
                                        10, // minimum output for movement (out of 127)
                                        1.019 // curve gain (a - value)
);

// Chassis
inline lemlib::Chassis chassis(driveTrain, lateral_controller, angular_controller, odom, &throttleCurve, &steerCurve);