#pragma once

#include <Hardware/HardwareManager.h>
#include <Hardware/IOMap.h>
#include <Basic/Mechanism.h>
#include <Basic/Feedback.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/current.h>
#include <wpi/numbers>

class SwerveModule : public Mechanism {
public:
    SwerveModule(unsigned driveID, unsigned turningID, unsigned canCoderID, bool driveInverted);
    ~SwerveModule();

    void sendFeedback(std::size_t moduleIndex);
    void doPersistentConfiguration() override;

    /**
     * Stop all motors.
     */
    void stop();

    /**
     * Sets the state of the swerve module. (Velocity and angle).
     */
    void setState(frc::SwerveModuleState state);

    /**
     * Returns the current state of the swerve module (Velocity and angle).
     */
    frc::SwerveModuleState getState();

    /**
     * Returns the raw rotation of the absolute turning encoder.
     */
    units::radian_t getRawRotation();

    /**
     * Sets the angle of the swerve module.
     */
    void setTurningMotor(units::radian_t angle);

    /**
     * Applies an offset to the turning magnetic encoder.
     */
    void setOffset(units::radian_t offset);

    /**
     * Sets the idle mode of the drive motor controller.
     */
    void setIdleMode(ThunderMotorController::IdleMode idleMode);

private:
    /**
     * Sets the velocity of the drive motor.
     */
    void setDriveMotor(units::meters_per_second_t velocity);
    
    /**
     * Configure motors to power-on states
     */
    void configureMotors();

    /**
     * Returns the relative rotation of the module (Rotations of the NEO 550).
     */
    double getRelativeRotation();

    /**
     * Returns the absolute rotation of the module (CANCoder encoder value).
     */
    frc::Rotation2d getAbsoluteRotation();

    /**
     * Returns the raw value of the drive motor's encoder (rotations).
     */
    double getRawDriveEncoder();
    
    /**
     * Returns the current velocity of the drive motor (meters per second).
     */
    units::meters_per_second_t getDriveVelocity();

    // The drive motor.
    HardwareManager::SwerveDriveMotor driveMotor;

    // The turning motor.
    HardwareManager::SwerveTurningMotor turningMotor;

    // The absolute encoder.
    HardwareManager::SwerveTurningEncoder turningAbsEncoder;

    // The offset of the CANCoders.
    units::radian_t canCoderOffset = 0_rad;

    // The target rotation of the swerve module.
    units::degree_t targetRotation = 0_deg;

    // Whether the drive motor is inverted.
    bool driveInverted = false;
};