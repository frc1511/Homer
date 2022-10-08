#include <Drive/Drive.h>

// The file which magnetic encoder offsets are stored on the RoboRIO.
#define ENCODER_OFFSETS_PATH "/home/lvuser/magnetic_encoder_offsets.txt"

// The maximum velocity during manual control.
#define DRIVE_MANUAL_MAX_VEL 1.5_mps

// The maximum angular velocity during manual control.
#define DRIVE_MANUAL_MAX_ANG_VEL 270_deg_per_s

Drive::Drive(Limelight* limelight)
: limelight(limelight), driveController(
    [&]() -> frc::HolonomicDriveController {
        // Set the angular PID controller range from -180 to 180 degrees.
        thetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        // Setup the drive controller with the individual axis PID controllers.
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

    // Enable the trajectory drive controller.
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

    ManualControlData lastManualData(manualData);
    // Reset the manual control data.
    manualData = {};

    // Reset the rate limiters to 0.
    driveRateLimiter.Reset(0_mps);
    turnRateLimiter.Reset(0_rad_per_s);

    // This seems to be necessary. Don't ask me why.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    if (mode == MatchMode::DISABLED) {
        /**
         * Coast all motors in disabled (good for transportation, however can
         * lead to some runaway robots).
         */
        setIdleMode(ThunderCANMotorController::IdleMode::COAST);

        teleopTimer.Stop();
        trajectoryTimer.Stop();

        // If the drivetrain movement was being recorded before being disabled.
        if (lastManualData.flags & ControlFlag::RECORDING) {
            /**
             * Export the recorded trajectory to a CSV file on the RoboRIO so
             * it can be examined and/or played back later.
             */
            trajectoryRecorder.writeToCSV(RECORDED_TRAJ_PATH);

            // Reload the recorded trajectory from the file.
            reloadRecordedTrajectory();
        }
    }
    else {
        // Brake all motors when enabled to help counteract pushing.
        setIdleMode(ThunderCANMotorController::IdleMode::BRAKE);

        /**
         * Calibrate the IMU if not already calibrated. This will cause the
         * robot to pause for 4 seconds while it waits for it to calibrate, so
         * the IMU should always be calibrated before the match begins.
         */
        if (!isIMUCalibrated()) {
            calibrateIMU();
        }

        // Reset the teleop timer.
        teleopTimer.Reset();
        lastTeleopTime = 0_s;

        if (mode == MatchMode::TELEOP) {
            teleopTimer.Start();

            // Clear the recorded trajectory.
            trajectoryRecorder.clear();
        }

        // Reset the trajectory timer.
        trajectoryTimer.Reset();

        if (mode == MatchMode::AUTO) {
            trajectoryTimer.Start();

            // Clear the motion profile CSV file.
            trajectoryMotionFile.clear();
            
            // Write the header of the CSV file.
            trajectoryMotionFile << "time,x_pos,y_pos,dest_x_pos,dest_y_pos,vel_x,vel_y,vel_ang,ang,dest_ang\n";
        }

        // Reset the position and rotation on the field.
        resetOdometry();
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

void Drive::runTrajectory(const Trajectory& _trajectory, const std::map<u_int32_t, Action*>& actionMap) {
    driveMode = DriveMode::TRAJECTORY;
    // Set the trajectory.
    trajectory = &_trajectory;

    // Set the initial action.
    trajectoryActionIter = trajectory->getActions().cbegin();

    trajectoryActions = &actionMap;

    // Reset done trajectory actions.
    doneTrajectoryActions.clear();

    // Reset the trajectory timer.
    trajectoryTimer.Reset();
    trajectoryTimer.Start();

    // Get the initial pose of the robot.
    frc::Pose2d initialPose(trajectory->getInitialPose());
    // Adjust the rotation because everything about this robot is 90 degrees off D:
    initialPose = frc::Pose2d(initialPose.X(), initialPose.Y(), (initialPose.Rotation().Degrees() - 90_deg));

    // Reset the odometry to the initial pose.
    resetOdometry(initialPose);
}

bool Drive::isFinished() const {
    // Stopped is as 'finished' as it gets I guess.
    return driveMode == DriveMode::STOPPED;
}

void Drive::calibrateIMU() {
    imu.calibrate();

    /**
     * Sleep the current thread manually because the IMU library doesn't
     * do it automatically for some reason D:
     */
    sleep(4);
    
    imuCalibrated = true;

    resetOdometry();
}

bool Drive::isIMUCalibrated() {
    return imuCalibrated;
}

void Drive::resetOdometry(frc::Pose2d pose) {
    /**
     * Resets the position and rotation of the robot to a given pose
     * while ofsetting for the IMU's recorded rotation.
     */
    odometry.ResetPosition(pose, getRotation());
}

frc::Pose2d Drive::getPose() {
    return odometry.GetPose();
}

frc::Rotation2d Drive::getRotation() {
    // The raw rotation from the IMU.
    units::degree_t imuAngle = imu.getAngle();

    return frc::Rotation2d(imuAngle);
}

void Drive::reloadRecordedTrajectory() {
    recordedTrajectory = Trajectory(RECORDED_TRAJ_PATH);
}

void Drive::updateOdometry() {
    /**
     * Using the rotation of the robot and the state (velocity
     * and rotation) of each swerve module, the odometry class is
     * able to calculate the robot's approximate position and
     * rotation on the field.
     */
    odometry.Update(getRotation(),
        swerveModules.at(0)->getState(),
        swerveModules.at(1)->getState(),
        swerveModules.at(2)->getState(),
        swerveModules.at(3)->getState()
    );
}

void Drive::execStopped() {
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Just for feedback.
    targetPose = getPose();

    // Put the drivetrain into brick mode if the flag is set.
    if (manualData.flags & ControlFlag::BRICK) {
        makeBrick();
    }

    // If recording, record the current state of the robot.
    if (manualData.flags & ControlFlag::RECORDING) {
        record_state();
    }
}

void Drive::execManual() {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel = manualData.xPct * DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel = manualData.yPct * DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t angVel = manualData.angPct * DRIVE_MANUAL_MAX_ANG_VEL;

    // The velocity of the robot using the component velocities.
    units::meters_per_second_t vel = units::math::hypot(xVel, yVel);

    // The heading of the robot's velocity.
    units::radian_t head = units::math::atan2(yVel, xVel);

    // Adjust the velocity using the configured acceleration and deceleration limits.
    vel = driveRateLimiter.Calculate(vel);

    // Calculate the new component velocities.
    xVel = units::math::cos(head) * vel;
    yVel = units::math::sin(head) * vel;

    // Adjust the angular velocity using the configured acceleration and deceleration limits.
    angVel = turnRateLimiter.Calculate(angVel);

    frc::Pose2d currPose(getPose());

    /**
     * Vice grip allows for the robot to remain aligned with a vision target and ignore
     * manual angular velocity control.
     */
    if (manualData.flags & ControlFlag::VICE_GRIP) {
        units::degree_t angleToTurn = -limelight->getAngleHorizontal();
        
        units::radian_t currentRotation(currPose.Rotation().Radians());

        // Reset the PID controller.
        static bool firstRun = true;
        if (firstRun) {
            thetaPIDController.Reset(currentRotation);
            firstRun = false;
        }

        /**
         * Calculate the angular velocity based on the error between the
         * current rotation and the target rotation.
         */
        angVel = units::radians_per_second_t(
            thetaPIDController.Calculate(currentRotation, currentRotation + angleToTurn)
        );
    }

    frc::ChassisSpeeds velocities;
    
    // Generate chassis speeds depending on the control mode.
    if (manualData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, angVel, currPose.Rotation());
    }
    else {
        // Chassis speeds are robot-centric.
        velocities = { xVel, yVel, angVel };
    }

    // Just for feedback.
    targetPose = currPose;

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);

    // If recording, record the current state of the robot.
    if (manualData.flags & ControlFlag::RECORDING) {
        record_state();
    }
}

void Drive::execTrajectory() {
    units::second_t time(trajectoryTimer.Get());

    int actionRes = 0;
    bool execAction = false;

    // If we've got another action to go.
    if (trajectoryActionIter != trajectory->getActions().cend()) {
        const auto& [action_time, actions] = *trajectoryActionIter;

        // Check if it's time to execute the action.
        if (time >= action_time) {
            execAction = true;

            // Loop through the available actions.
            for (auto it(trajectoryActions->cbegin()); it != trajectoryActions->cend(); ++it) {
                const auto& [id, action] = *it;

                // Narrow the list down to only actions that have not been completed yet.
                if (std::find(doneTrajectoryActions.cbegin(), doneTrajectoryActions.cend(), id) == doneTrajectoryActions.cend()) {
                    // If the action's bit is set in the bit field.
                    if (actions & id) {
                        // Execute the action.
                        Action::Result res = action->process();

                        // If the action has completed.
                        if (res == Action::Result::DONE) {
                            // Remember that it's done.
                            doneTrajectoryActions.push_back(id);
                        }

                        actionRes += res;
                    }
                }
            }
        }
    }

    // Stop the trajectory because an action is still running.
    if (actionRes) {
        trajectoryTimer.Stop();
    }
    // Continue/Resume the trajectory because the actions are done.
    else {
        // Increment the action if an action was just finished.
        if (execAction) {
            ++trajectoryActionIter;
            doneTrajectoryActions.clear();
        }
        trajectoryTimer.Start();
    }

    // If the trajectory is done, then stop it.
    if (time > trajectory->getDuration() && driveController.AtReference()) {
        driveMode = DriveMode::STOPPED;
        return;
    }

    // Sample the trajectory at the current time for the desired state of the robot.
    Trajectory::State state(trajectory->sample(time));

    // Don't be moving if an action is being worked on.
    if (actionRes) {
        state.velocity = 0_mps;
    }

    // Adjust the rotation because everything about this robot is 90 degrees off D:
    state.rotation = state.rotation.Degrees() - 90_deg;

    // The current pose of the robot.
    frc::Pose2d currentPose(getPose());

    // The desired position delta of the robot.
    units::meter_t dx(state.xPos - currentPose.X()),
                   dy(state.yPos - currentPose.Y());

    // The angle at which the robot should be driving at.
    frc::Rotation2d heading(units::math::atan2(dy, dx));

    /**
     * Calculate the chassis velocities based on the error between the current
     * pose and the desired pose.
     */
    frc::ChassisSpeeds velocities(
        driveController.Calculate(
            currentPose,
            frc::Pose2d(state.xPos, state.yPos, heading),
            state.velocity,
            state.rotation
        )
    );

    // Keep target pose for feedback.
    targetPose = frc::Pose2d(state.xPos, state.yPos, state.rotation);

    // Log motion to CSV file for debugging.
    trajectoryMotionFile << time.value() << ','
                         << currentPose.X().value() << ','
                         << currentPose.Y().value() << ','
                         << targetPose.X().value() << ','
                         << targetPose.Y().value() << ','
                         << velocities.vx.value() << ','
                         << velocities.vy.value() << ','
                         << velocities.omega.value() << ','
                         << currentPose.Rotation().Radians().value() << ','
                         << state.rotation.Radians().value() << '\n';

    // Make the robot go vroom :D
    setModuleStates(velocities);
}

void Drive::record_state() {
    units::second_t currTime(teleopTimer.Get());
    frc::Pose2d currPose(getPose());

    // Add the state to the trajectory recorder.
    trajectoryRecorder.addState(
        // The delta time between logged states.
        currTime - lastTeleopTime,
        // Add 90 degrees because everything on this robot is 90 degrees off.
        frc::Pose2d(currPose.X(), currPose.Y(), currPose.Rotation().Degrees() + 90_deg)
    );

    lastTeleopTime = currTime;
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
    /**
     * Only allow configuration in crater mode because configuring them
     * accidentally would be D:
     */
    if (!settings.isCraterMode) {
        return;
    }

    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        /**
         * Get the current rotation of the swerve modules to save as the
         * forward angle of the module. This is a necessary step because
         * when installed, each magnetic encoder has its own offset.
         */
        units::radian_t angle(swerveModules.at(i)->getRawRotation());
        
        // Set the current angle as the new offset angle.
        offsets.at(i) = angle;
    }

    // Write the new offsets to the offsets file.
    writeOffsetsFile();

    // Apply the new offsets to the swerve modules.
    applyOffsets();
}

bool Drive::readOffsetsFile() {
    // Open the file.
    std::ifstream file(ENCODER_OFFSETS_PATH);
    
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
        
        // Set the offset in the array to the parsed number (saved in radians).
        offsets.at(i) = units::radian_t(num);
        
        // Increment the index.
        ++i;
    }

    return true;
}

void Drive::writeOffsetsFile() {
    // Open the file (Will create a new file if it does not already exist).
    std::ofstream offsetsFile(ENCODER_OFFSETS_PATH);
    
    // Clear the contents of the file.
    offsetsFile.clear();

    // Write each offset to the file (in radians).
    for (units::radian_t offset : offsets) {
        offsetsFile << offset.value() << '\n';
    }
}

void Drive::applyOffsets() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        /**
         * The offsets will be 90 degrees off because EVERYTHING about
         * this robot is 90 degrees off!
         */
        swerveModules.at(i)->setOffset(offsets.at(i) - 90_deg);
    }
}

void Drive::setIdleMode(ThunderCANMotorController::IdleMode mode) {
    for (SwerveModule* module : swerveModules) {
        module->setIdleMode(mode);
    }
}

void Drive::setModuleStates(frc::ChassisSpeeds speeds) {
    // Store velocities for feedback.
    chassisSpeeds = speeds;

    // Generate individual module states using the chassis velocities.
    wpi::array<frc::SwerveModuleState, 4> moduleStates(kinematics.ToSwerveModuleStates(speeds));
    
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

    frc::Pose2d pose(getPose());

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
    Feedback::sendBoolean("Drive", "recording", manualData.flags & ControlFlag::RECORDING);

    // Feedback for the motion profile.
    Feedback::sendDouble("DriveCSV", "x_pos", pose.X().value());
    Feedback::sendDouble("DriveCSV", "y_pos", pose.Y().value());
    Feedback::sendDouble("DriveCSV", "t_x_pos", targetPose.X().value());
    Feedback::sendDouble("DriveCSV", "t_y_pos", targetPose.Y().value());
    Feedback::sendDouble("DriveCSV", "x_vel", chassisSpeeds.vx.value());
    Feedback::sendDouble("DriveCSV", "y_vel", chassisSpeeds.vy.value());
    Feedback::sendDouble("DriveCSV", "ang_vel", chassisSpeeds.omega.value());
    Feedback::sendDouble("DriveCSV", "ang", pose.Rotation().Radians().value());
    Feedback::sendDouble("DriveCSV", "t_ang", targetPose.Rotation().Radians().value());
}
