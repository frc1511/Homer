#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <Drive/SwerveModule.h>
#include <Drive/Trajectory.h>
#include <Basic/Feedback.h>

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <wpi/numbers>
#include <array>
#include <fstream>
#include <frc/Timer.h>

// The width of the robot.
#define ROBOT_WIDTH 0.362_m
// The length of the robot.
#define ROBOT_LENGTH 0.66_m

#define DRIVE_AUTO_MAX_ANGULAR_VELOCITY 3.14_rad_per_s
#define DRIVE_AUTO_MAX_ANGULAR_ACCELERATION (3.14_rad_per_s_sq)

class Drive : public Mechanism {
public:
    Drive();
    ~Drive();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    enum ControlFlag {
        NONE = 0,
        FIELD_CENTRIC = 1 << 0,
        VICE_GRIP     = 1 << 1,
        BRICK         = 1 << 2,
    };

    /**
     * Controls the speeds of drivetrain using percentages of the max speed.
     * (The direction of the velocities is dependant on the control type).
     * 
     * Positive xPct   -> Move right,             Negative xPct   -> Move left.
     * Positive yPct   -> Move forward,           Negative yPct   -> Move backward.
     * Positive angPct -> Turn counter-clockwise, Negative angPct -> Turn clockwise.
     * 
     * Control flags are used to control the behavior of the drivetrain.
     */
    void manualControl(double xPct, double yPct, double angPct, unsigned flags);

    /**
     * Runs a trajectory.
     */
    void runTrajectory(const Trajectory& trajectory);

    /**
     * Returns whether the current process is finished or not.
     */
    bool isFinished() const;

    /**
     * Resets the rotation of the robot tracked by odometry.
     */
    void zeroRotation();

    /**
     * Calibrates the IMU.
     */
    void calibrateIMU();

    /**
     * Returns whether the IMU is calibrated.
     */
    bool isIMUCalibrated();

    /**
     * Resets the position and rotation of the drivetrain on the field.
     */
    void resetOdometry(frc::Pose2d pose = frc::Pose2d());

    /**
     * Returns the coordinates of the robot on the field.
     */
    frc::Pose2d getPose();

    /**
     * Returns the rotation of the robot on the field.
     */
    frc::Rotation2d getRotation();

private:
    /**
     * Updates the position and rotation of the drivetrain on the field.
     */
    void updateOdometry();

    /**
     * Executes the process for when the drivetrain is stopped.
     */
    void execStopped();

    /**
     * Executes the process for when the drivetrain is in manual control.
     */
    void execManual();

    /**
     * Executes the process for when the drivetrain is running a trajectory.
     */
    void execTrajectory();

    /**
     * Puts the drivetrain into brick mode (all modules turned towards the center).
     */
    void makeBrick();

    /**
     * Applies the current rotation of the swerve modules as the offset of the
     * magnetic encoders.
     */
    void configMagneticEncoders();

    /**
     * Reads the magnetic encoder offsets file.
     */
    bool readOffsetsFile();

    /**
     * Writes the current magnetic encoder offsets into the file.
     */
    void writeOffsetsFile();

    /**
     * Applies the current magnetic encoder offsets to the swerve modules.
     */
    void applyOffsets();

    /**
     * Sets the idle mode of the drive motors.
     */
    void setIdleMode(ThunderMotorController::IdleMode mode);

    /**
     * Sets the velocities of the drivetrain.
     */
    void setModuleStates(frc::ChassisSpeeds speeds);

    // The IMU.
    HardwareManager::DriveIMU imu;

    // Whether the IMU is calibrated.
    bool imuCalibrated = false;

    // The locations of the swerve modules on the robot.
    std::array<frc::Translation2d, 4> locations {
        frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front left.
        frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back left.
        frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back right.
        frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front right.
    };

    // The swerve modules on the robot.
    std::array<SwerveModule*, 4> swerveModules {
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_FL, CAN_SWERVE_ROT_MOTOR_FL, CAN_SWERVE_CAN_CODER_FL, true),
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_BL, CAN_SWERVE_ROT_MOTOR_BL, CAN_SWERVE_CAN_CODER_BL, true),
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_BR, CAN_SWERVE_ROT_MOTOR_BR, CAN_SWERVE_CAN_CODER_BR, true),
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_FR, CAN_SWERVE_ROT_MOTOR_FR, CAN_SWERVE_CAN_CODER_FR, false),
    };

    // The magnetic encoder offsets of the swerve modules.
    std::array<units::radian_t, 4> offsets { 0_rad, 0_rad, 0_rad, 0_rad };

    /**
     * The helper class that can be used to convert chassis speeds into swerve
     * module states.
     */
    frc::SwerveDriveKinematics<4> kinematics { locations };

    /**
     * The class that will handle tracking the position of the robot on the
     * field during the match.
     */
    frc::SwerveDriveOdometry<4> odometry { kinematics, getRotation() };

    enum class DriveMode {
        STOPPED,
        MANUAL,
        TRAJECTORY,
    };

    // The current drive mode.
    DriveMode driveMode = DriveMode::STOPPED;

    struct ManualControlData {
        double xPct = 0;
        double yPct = 0;
        double angPct = 0;
        unsigned flags = ControlFlag::NONE;
    };

    // The data concerning manual control.
    ManualControlData manualData {};

    // The trajectory that is currently being run.
    const Trajectory* trajectory = nullptr;

    frc::Timer trajectoryTimer;

    // PID Controller for X and Y axis drivetrain movement.
    frc::PIDController xPIDController { 1.0, 0.0, 0.01 },
                       yPIDController { 1.0, 0.0, 0.01 };

    // PID Controller for the angular drivetrain movement.
    frc::ProfiledPIDController<units::radians> thetaPIDController {
        2.5, 0.0, 0.0,
        frc::TrapezoidProfile<units::radians>::Constraints(DRIVE_AUTO_MAX_ANGULAR_VELOCITY, DRIVE_AUTO_MAX_ANGULAR_ACCELERATION)
    };

    // The drive controller that will handle the drivetrain movement.
    frc::HolonomicDriveController driveController;

    frc::ChassisSpeeds chassisSpeeds { 0_mps, 0_mps, 0_rad_per_s };
    frc::Pose2d targetPose;

    std::ofstream trajectoryMotionFile { "/home/lvuser/trajectory_motion.csv" };
};
