#pragma once

#include <Wrappers/GameController/PS4Controller.h>
#include <Wrappers/GameController/XboxController.h>

#include <Wrappers/MotorController/CANMotorController.h>
#include <Wrappers/MotorController/Rev/CANSparkMax.h>
#include <Wrappers/MotorController/CTRE/CANTalonFX.h>

#include <Wrappers/MagneticEncoder/CANMagneticEncoder.h>
#include <Wrappers/MagneticEncoder/CTRE/CANCoder.h>

#include <Wrappers/IMU/IMU.h>
#include <Wrappers/IMU/ADIS16470IMU.h>

#include <Hardware/IOMap.h>

class HardwareManager {
public:

// Test board hardware.
#ifdef TEST_BOARD
    using SwerveDriveMotor = ThunderCANMotorController;
    using SwerveTurningMotor = ThunderCANMotorController;
    using SwerveTurningEncoder = ThunderCANMagneticEncoder;
    using DriveIMU = ThunderIMU;

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
