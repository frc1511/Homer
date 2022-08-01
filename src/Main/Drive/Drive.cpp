#include <Drive/Drive.h>

// The file in which magnetic encoder offsets are stored on the RoboRIO.
#define ENCODER_OFFSETS_FILE_NAME "/home/lvuser/magnetic_encoder_offsets.txt"

// The maximum velocity during manual control.
#define DRIVE_MANUAL_MAX_VELOCITY 1_mps
// The maximum angular velocity during manual control.
#define DRIVE_MANUAL_MAX_ANGULAR_VELOCITY 90_deg_per_s

// The angle at which vice grip doesn't care anymore.
#define ROTATION_THRESHOLD 1_deg

Drive::Drive()
: driveController([&]() -> frc::HolonomicDriveController {
        thetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        return frc::HolonomicDriveController(xPIDController, yPIDController, thetaPIDController);
    } ()) {

    // 4s is good?
    imu.configCalTime(ThunderIMU::CalibrationTime::_4s);
    imu.setYawAxis(ThunderIMU::Axis::Z);
    imu.reset();

    // Apply the magnetic encoder offsets if the config file exists.
    if (readOffsetsFile()) {
        applyOffsets();
    }

    driveController.SetEnabled(true);
}

Drive::~Drive() {
    for (SwerveModule* module : swerveModules) {
        delete module;
    }
}

void Drive::doPersistentConfiguration() {
    for (SwerveModule* module : swerveModules) {
        module->doPersistentConfiguration();
    }
    configMagneticEncoders();
}

void Drive::resetToMode(MatchMode mode) {
    driveMode = DriveMode::STOPPED;
    manualData = {};

    // This seems to be necessary.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    if (mode == MatchMode::DISABLED) {
        // Brake all motors when disabled (no runaway robots anymore).
        setIdleMode(ThunderMotorController::IdleMode::BRAKE);
    }
    else {
        // Brake all motors when enabled to help counteract pushing.
        setIdleMode(ThunderMotorController::IdleMode::BRAKE);

        // Calibrate the IMU if not already calibrated.
        if (!isIMUCalibrated()) {
            calibrateIMU();
        }
    }
}

void Drive::process() {
    updateOdometry();

    switch (driveMode) {
        case DriveMode::STOPPED:
            execStopped();
            break;
        case DriveMode::MANUAL:
            execManual();
            break;
        case DriveMode::TRAJECTORY:
            execTrajectory();
            break;
    }
}

void Drive::manualControl(double xPct, double yPct, double angPct, unsigned flags) {
    // Stop the robot in brick mode no matter what.
    if (flags & ControlFlag::BRICK) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot is still doing stuff when vice gripping.
    else if (flags & ControlFlag::VICE_GRIP) {
        driveMode = DriveMode::MANUAL;
    }
    // The robot isn't being told to do anything, sooo.... stop??
    else if ((!xPct && !yPct && !angPct)) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot is being told to do stuff, so start doing stuff.
    else {
        driveMode = DriveMode::MANUAL;
    }

    manualData = { xPct, yPct, angPct, flags };
}

<<<<<<< HEAD
void Drive::runTrajectory(const char* path) {
    std::string file_str;
    {
        std::ifstream file(path);
        if (!file) return;
        file_str = std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }

    std::string::const_iterator file_iter = file_str.cbegin();

    auto count = [&]() -> std::size_t {
        std::size_t n = 0;
        while (file_iter != file_str.end() && *file_iter != '\n' && *file_iter != ',' && *file_iter != '}') {
        n++;
        file_iter++;
        }
        return n;
    };

    auto get_str = [&]() -> std::string {
        std::string::const_iterator start = file_iter;
        std::size_t n = count();
        return std::string(start, start + n);
    };

    
    file_iter += 29;

    while (file_iter != file_str.cend()) {
        float time = std::stof(get_str()); ++file_iter;
        float x_pos = std::stof(get_str()); ++file_iter;
        float y_pos = std::stof(get_str()); ++file_iter;
        float velocity = std::stof(get_str()); ++file_iter;

        trajectoryPoints.push_back(TrajectoryPoint{time, x_pos, y_pos, velocity});
    }

    trajectoryTimer.Start();
    driveMode = DriveMode::TRAJECTORY;
=======
void Drive::runTrajectory(const Trajectory& _trajectory) {
    driveMode = DriveMode::TRAJECTORY;
    trajectory = &_trajectory;

    trajectoryTimer.Reset();
    trajectoryTimer.Start();

    resetOdometry(trajectory->getInitialPose());
}

bool Drive::isFinished() const {
    return driveMode == DriveMode::STOPPED;
>>>>>>> 6ad1376ea2cd21dfe435c6b89c87897f59e3ca05
}

void Drive::zeroRotation() {
    // imu.reset();
    resetOdometry(getPose());
}

void Drive::calibrateIMU() {
    imu.calibrate();
    // Sleep the current thread manually because the IMU library doesn't
    // do it automatically for some reason D:
    sleep(4);
    
    imuCalibrated = true;
}

bool Drive::isIMUCalibrated() {
    return imuCalibrated;
}

void Drive::resetOdometry(frc::Pose2d pose) {
    odometry.ResetPosition(pose, getRotation());
}

frc::Pose2d Drive::getPose() {
    return odometry.GetPose();
}

frc::Rotation2d Drive::getRotation() {
    units::angle::degree_t imuAngle = imu.getAngle();

    // Make rotation be between 0 and 360 degrees.
    units::radian_t rotation(units::math::fmod(imuAngle, 360_deg));

    return frc::Rotation2d(rotation);
}

void Drive::updateOdometry() {
    // Track the position and rotation of the robot on the field.
    odometry.Update(getRotation(),
        swerveModules.at(0)->getState(),
        swerveModules.at(1)->getState(),
        swerveModules.at(2)->getState(),
        swerveModules.at(3)->getState());
}

void Drive::execStopped() {
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Put the drivetrain into brick mode if the flag is set.
    if (manualData.flags & ControlFlag::BRICK) {
        makeBrick();
    }
}

void Drive::execManual() {
    units::meters_per_second_t xVel = manualData.xPct * DRIVE_MANUAL_MAX_VELOCITY;
    units::meters_per_second_t yVel = manualData.yPct * DRIVE_MANUAL_MAX_VELOCITY;
    units::radians_per_second_t angVel = 0_rad_per_s;

    /**
     * Vice grip allows for the robot to remain aligned with a vision target and ignore
     * manual angular velocity control.
     */
    if (manualData.flags & ControlFlag::VICE_GRIP) {
        // units::degree_t angleToTurn = -limelight->getAngleHorizontal();
        units::degree_t angleToTurn = 0_deg;
        
        units::radian_t currentRotation(getRotation().Radians());

        static bool firstRun = true;
        if (firstRun) {
            thetaPIDController.Reset(currentRotation);
            firstRun = false;
        }

        angVel = units::radians_per_second_t(
            thetaPIDController.Calculate(currentRotation, currentRotation + angleToTurn)
        );
    }
    else {
        angVel = manualData.angPct * DRIVE_MANUAL_MAX_ANGULAR_VELOCITY;
    }

    frc::ChassisSpeeds velocities;
    
    // Generate chassis speeds depending on the control mode.

    if (manualData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, angVel, getRotation());
    }
    else {
        velocities = { xVel, yVel, angVel };
    }

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);
}

void Drive::execTrajectory() {
<<<<<<< HEAD
    decltype(trajectoryPoints)::const_iterator pt_it;

    for (std::vector<TrajectoryPoint>::const_iterator it = trajectoryPoints.cbegin(); it != trajectoryPoints.cend(); ++it) {
        double time = trajectoryTimer.Get().value();
        if (time >= it->time && (it == trajectoryPoints.cend() - 1 || time < (it + 1)->time)) {
            pt_it = it;
            break;
        }
    }

    
=======
    units::second_t time = trajectoryTimer.Get();

    if (time > trajectory->getDuration() && driveController.AtReference()) {
        driveMode = DriveMode::STOPPED;
        return;
    }

    // Get the current state of the robot on the trajectory.
    Trajectory::State state = trajectory->sample(time);

    // The current pose of the robot.
    frc::Pose2d currentPose = getPose();

    // The desired change in position of the robot.
    units::meter_t dx(state.xPos - currentPose.X()),
                   dy(state.yPos - currentPose.Y());

    // The angle which the robot should be driving at.
    frc::Rotation2d heading(units::math::atan2(dy, dx));

    frc::ChassisSpeeds velocities(
        driveController.Calculate(
            currentPose,
            frc::Pose2d(state.xPos, state.yPos, heading),
            state.velocity,
            state.rotation
        )
    );

    // Make the robot go vroom :D
    setModuleStates(velocities);
>>>>>>> 6ad1376ea2cd21dfe435c6b89c87897f59e3ca05
}

void Drive::makeBrick() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        units::degree_t angle;
        // If the index is even.
        if (i % 2 == 0) {
            angle = -45_deg;
        }
        // If the index is odd.
        else {
            angle = 45_deg;
        }
        
        // Stop the robot. It should already be stopped tho.
        driveMode = DriveMode::STOPPED;
        
        // Turn the swerve module to point towards the center of the robot.
        swerveModules.at(i)->setTurningMotor(angle);
    }
}

void Drive::configMagneticEncoders() {
    // Only allow configuration in crater mode because configuring them
    // accidentally would be D:
    if (!settings.isCraterMode) {
        return;
    }

    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        // Get the current rotation of the swerve modules so that we can know
        // the angle read when the module is rotated forwards, which is
        // should be when the configuration is done.
        units::radian_t angle(swerveModules.at(i)->getRawRotation());
        
        // Set that as the new offset angle.
        offsets.at(i) = angle;
    }

    // Write the new offsets to the offsets file.
    writeOffsetsFile();

    // Apply the new offsets to the swerve modules.
    applyOffsets();
}

bool Drive::readOffsetsFile() {
    // Open the file.
    std::ifstream file(ENCODER_OFFSETS_FILE_NAME);
    
    // Make sure the file exists.
    if (!file) {
        return false;
    }

    std::size_t i = 0;
    std::string line;
    // Loop through each line of the file.
    while (getline(file, line) && i <= 3) {
        // Convert the line string to a number.
        double num = std::atof(line.c_str());
        
        // Make sure the number is not 0 (We don't really care about 0s, plus
        // atof() returns 0 when there is an error parsing the string)
        if (num) {
            // Set the offset in the array to the parsed number.
            offsets.at(i) = units::radian_t(num);
        }
        
        // Increment the index.
        ++i;
    }

    return true;
}

void Drive::writeOffsetsFile() {
    // Open the file (Will create a new file if it does not already exist).
    std::ofstream file(ENCODER_OFFSETS_FILE_NAME);
    
    // Clear the contents of the file.
    file.clear();

    // Write each offset to the file.
    for (units::radian_t offset : offsets) {
        file << offset.value() << '\n';
    }
}

void Drive::applyOffsets() {
    for (unsigned i = 0; i < swerveModules.size(); i++) {
        // The offsets will be 90 degrees off because 0 degeres on a
        // graph translates to the drivetrain moving right.
        swerveModules.at(i)->setOffset(offsets.at(i) - 90_deg);
    }
}

void Drive::setIdleMode(ThunderMotorController::IdleMode mode) {
    for (SwerveModule* module : swerveModules) {
        module->setIdleMode(mode);
    }
}

void Drive::setModuleStates(frc::ChassisSpeeds speeds) {
    // Generate module states using the chassis velocities. This is the magic
    // function of swerve drive.
    wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(speeds);
    
    // Recalculate the wheel velocities relative to the max speed. Actually
    // does absolutely nothing in this situation.
    kinematics.DesaturateWheelSpeeds(&moduleStates, DRIVE_MANUAL_MAX_VELOCITY);
    
    // Set the states of the individual modules.
    for(std::size_t i = 0; i < swerveModules.size(); i++) {
      swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

void Drive::sendFeedback() {
    // Module feedback.
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->sendFeedback(i);
    }

    frc::Pose2d pose = getPose();

    // Drive feedback.
    Feedback::sendDouble("Drive", "x position (m)", pose.X().value());
    Feedback::sendDouble("Drive", "y position (m)", pose.Y().value());
    Feedback::sendDouble("Drive", "rotation (deg)", getRotation().Degrees().value());

    Feedback::sendDouble("Drive", "manual x%", manualData.xPct);
    Feedback::sendDouble("Drive", "manual y%", manualData.yPct);
    Feedback::sendDouble("Drive", "manual angular%", manualData.angPct);

    Feedback::sendBoolean("Drive", "field centric", manualData.flags & ControlFlag::FIELD_CENTRIC);
    Feedback::sendBoolean("Drive", "vice grip", manualData.flags & ControlFlag::VICE_GRIP);
    Feedback::sendBoolean("Drive", "brick", manualData.flags & ControlFlag::BRICK);
}