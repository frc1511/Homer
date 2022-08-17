#include <Drive/SwerveModule.h>

// The max amperage of the drive motors.
#define DRIVE_MAX_AMPERAGE 40_A
// The max amperage of the turning motors.
#define TURN_MAX_AMPERAGE 30_A

// The number of seconds it will take for the drive motors to ramp from idle to full throttle.
#define DRIVE_RAMP_TIME 1_s

// --- PID Values ---

#define DRIVE_P 0.00001
#define DRIVE_I 0
#define DRIVE_D 0
#define DRIVE_I_ZONE 0
#define DRIVE_FF 0.000187

#define TURN_P 0.1
#define TURN_I 0
#define TURN_D 0
#define TURN_I_ZONE 0
#define TURN_FF 0

// The coeffieient used to convert a radian value into rotations of the NEO 550 turning motor.
#define TURN_RADIAN_TO_ENCODER_FACTOR 10.1859

// Drive encoder value after one foot.
#define DRIVE_FOOT_TO_ENDODER_FACTOR 6.51

// The coefficient used to convert a distance in meters into a number of rotations of the NEO drive motors.
#define DRIVE_METER_TO_ENCODER_FACTOR (DRIVE_FOOT_TO_ENDODER_FACTOR * 3.28084)

// The coefficient used to convert rotations of the NEO drive motors into a distance traveled in meters.
#define DRIVE_ENCODER_TO_METER_FACTOR (1 / (DRIVE_METER_TO_ENCODER_FACTOR))


SwerveModule::SwerveModule(unsigned driveID, unsigned turningID, unsigned canCoderID, bool driveInverted)
: driveMotor(driveID), turningMotor(turningID), turningAbsEncoder(canCoderID), driveInverted(driveInverted) {
    
    configureMotors();

    // --- CANCoder configuration ---

    turningAbsEncoder.configFactoryDefault();
    // Set the range of the CANCoder to -180 to +180 degrees instead of 0 to 360 degrees.
    turningAbsEncoder.configAbsoluteSensorRange(ThunderCANMagneticEncoder::SensorRange::SIGNED_PLUS_MINUS_180);
}

SwerveModule::~SwerveModule() = default;

void SwerveModule::configureMotors() {
    // --- Drive motror config ---

    driveMotor.configFactoryDefault();

    // Set the idle mode to coast.
    driveMotor.setIdleMode(ThunderMotorController::IdleMode::COAST);

    // Amperage limiting.
    driveMotor.configSmartCurrentLimit(DRIVE_MAX_AMPERAGE);

    driveMotor.setInverted(driveInverted);

    // Ramping.
    driveMotor.configOpenLoopRamp(DRIVE_RAMP_TIME);
    driveMotor.configClosedLoopRamp(DRIVE_RAMP_TIME);

    // PID Values.
    driveMotor.configP(DRIVE_P);
    driveMotor.configI(DRIVE_I);
    driveMotor.configD(DRIVE_D);
    driveMotor.configIZone(DRIVE_I_ZONE);
    driveMotor.configFF(DRIVE_FF);

    // --- Turning motor config ---

    turningMotor.configFactoryDefault();

    // Coast when idle so that people can turn the module.
    turningMotor.setIdleMode(ThunderMotorController::IdleMode::COAST);

    // Amperage limiting.
    turningMotor.configSmartCurrentLimit(TURN_MAX_AMPERAGE);

    // It is not inverted!
    turningMotor.setInverted(true);

    // PID Values.
    turningMotor.configP(TURN_P);
    turningMotor.configI(TURN_I);
    turningMotor.configD(TURN_D);
    turningMotor.configIZone(TURN_I_ZONE);
    turningMotor.configFF(TURN_FF);
}

void SwerveModule::doPersistentConfiguration() {
    configureMotors();

    // Burn the current configuration into the motor controllers' flash memory.
    driveMotor.burnFlash();
    turningMotor.burnFlash();
}

void SwerveModule::stop() {
    turningMotor.set(ThunderMotorController::ControlMode::PERCENT_OUTPUT, 0.0);
    driveMotor.set(ThunderMotorController::ControlMode::PERCENT_OUTPUT, 0.0);
    driveMotor.setEncoderPosition(0.0);
}

void SwerveModule::setState(frc::SwerveModuleState targetState) {
    frc::SwerveModuleState currentState = getState();

    frc::SwerveModuleState optimizedState;

    // Turn off optimization in crater mode to help with configuration.
    if (settings.isCraterMode) {
        optimizedState = targetState;
    }
    else {
        /**
         * Optimize the target state by flipping motor directions and adjusting
         * rotations in order to turn the least amount of distance possible.
         */
        optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
    }

    /**
     * Only handle turning when the robot is actually driving (Stops the modules
     * from snapping back to 0 when the robot comes to a stop).
     */
    if(units::math::abs(optimizedState.speed) > 0.01_mps) {
        // Rotate the swerve module to the desired angle.
        setTurningMotor(optimizedState.angle.Radians());
    }
  
    // Set the drive motor's velocity.
    setDriveMotor(optimizedState.speed);
}

frc::SwerveModuleState SwerveModule::getState() {
    // The velocity and rotation of the swerve module.
    return { getDriveVelocity(), getAbsoluteRotation() };
}

units::radian_t SwerveModule::getRawRotation() {
    return turningAbsEncoder.getAbsolutePosition();
}

void SwerveModule::setTurningMotor(units::radian_t angle) {
    // Subtract the absolute rotation from the target rotation to get the angle to turn.
    units::radian_t angleDelta(angle - getAbsoluteRotation().Radians());
    
    /**
     * Fix the discontinuity problem by converting a -2π to 2π value into -π to π value.
     * If the value is above π rad or below -π rad...
     */
    if(units::math::abs(angleDelta).value() > wpi::numbers::pi) {
        int sign = std::signbit(angleDelta.value()) ? -1 : 1;
        
        // Subtract 2π rad, or add 2π rad depending on the sign.
        angleDelta = units::radian_t(angleDelta.value() - (2 * wpi::numbers::pi) * sign);
    }

    // Add back the absolute rotation to get the desired angle.
    targetRotation = angleDelta + getAbsoluteRotation().Degrees();
    
    // Convert the angle (radians) to a NEO 550 encoder value.
    double output = angleDelta.value() * TURN_RADIAN_TO_ENCODER_FACTOR;
    
    // Add the current relative rotation to get the position to reference.
    output += getRelativeRotation();

    // Set the PID reference to the desired position.
    turningMotor.set(ThunderMotorController::ControlMode::POSITION, output);
}

void SwerveModule::setOffset(units::radian_t offset) {
    canCoderOffset = offset;
}

void SwerveModule::setIdleMode(ThunderMotorController::IdleMode idleMode) {
    driveMotor.setIdleMode(idleMode);
}

void SwerveModule::setDriveMotor(units::meters_per_second_t velocity) {
    // Convert the velocity value (meters per second) into RPM.
    double rpm = velocity.value() * 60 * DRIVE_METER_TO_ENCODER_FACTOR;

    // Set the PID reference to the desired RPM.
    driveMotor.set(ThunderMotorController::ControlMode::VELOCITY, rpm);
}

double SwerveModule::getRelativeRotation() {
    return turningMotor.getEncoderPosition();
}

frc::Rotation2d SwerveModule::getAbsoluteRotation() {
    // The angle from the CANCoder.
    units::degree_t angle(getRawRotation());

    // Subtract the offset from the angle.
    angle -= canCoderOffset;

    return angle;
}

double SwerveModule::getRawDriveEncoder() {
    return driveMotor.getEncoderPosition();
}

units::meters_per_second_t SwerveModule::getDriveVelocity() {
    // Convert the RPM to a velocity value (meters per second).
    double mps = (driveMotor.getEncoderVelocity() / 60) * DRIVE_ENCODER_TO_METER_FACTOR;
    
    return units::meters_per_second_t(mps);
}

void SwerveModule::sendFeedback(std::size_t moduleIndex) {
    std::string i = std::to_string(moduleIndex);
    
    Feedback::sendDouble("Drive Module", "module " + i + " relative rotation", getRelativeRotation());
    Feedback::sendDouble("Drive Module", "module " + i + " raw rotation (deg)", units::degree_t(getRawRotation()).value());
    Feedback::sendDouble("Drive Module", "module " + i + " rotation (deg)", getAbsoluteRotation().Degrees().value());
    Feedback::sendDouble("Drive Module", "module " + i + " encoder offset (deg)", units::degree_t(canCoderOffset).value());
    Feedback::sendDouble("Drive Module", "module " + i + " target rotation (deg)", units::degree_t(targetRotation).value());
    Feedback::sendDouble("Drive Module", "module " + i + " drive encoder", getRawDriveEncoder());
    Feedback::sendDouble("Drive Module", "module " + i + " velocity (m/s)", getDriveVelocity().value());
    Feedback::sendDouble("Drive Module", "module " + i + " turning motor temp (F)", turningMotor.getTemperature().value());
    Feedback::sendDouble("Drive Module", "module " + i + " drive motor temp (F)", driveMotor.getTemperature().value());
    // hi jeff
}
