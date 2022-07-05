#pragma once

#include <Wrappers/GameController/PS4Controller.h>
#include <Wrappers/GameController/XBoxController.h>
#include <Wrappers/MotorController/Rev/CANSparkMax.h>
#include <Wrappers/MotorController/Empty.h>
#include <Wrappers/MagneticEncoder/CTRE/CANCoder.h>
#include <Wrappers/MagneticEncoder/CANEmpty.h>
#include <Wrappers/IMU/ADIS16470IMU.h>
#include <Wrappers/IMU/Empty.h>

class HardwareManager {
public:

// Test board hardware.
#ifdef TEST_BOARD
    using SwerveDriveMotor = ThunderMotorControllerEmpty;
    using SwerveTurningMotor = ThunderMotorControllerEmpty;
    using SwerveTurningEncoder = ThunderCANMagneticEncoderEmpty;
    using DriveIMU = thunder::IMUEmpty;

// Regular robot hardware.
#else
    using SwerveDriveMotor = ThunderCANSparkMax;
    using SwerveTurningMotor = ThunderCANSparkMax;
    using SwerveTurningEncoder = ThunderCANCoder;
    using DriveIMU = ThunderADIS16470IMU;
#endif

    using DriveGameController = ThunderPS4Controller;
    using AuxGameController = ThunderPS4Controller;
};