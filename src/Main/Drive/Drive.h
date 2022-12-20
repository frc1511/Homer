#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <Drive/SwerveModule.h>
#include <Vision/Limelight.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <Trajectory/Trajectory.h>
#include <Trajectory/TrajectoryRecorder.h>
#include <Basic/Feedback.h>
#include <Autonomous/Action.h>

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Timer.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <wpi/array.h>
#include <numbers>
#include <fstream>
#include <map>
#include <algorithm>
#include <filesystem>

// The width of the robot.
#define ROBOT_WIDTH 0.362_m
// The length of the robot.
#define ROBOT_LENGTH 0.66_m

// The maximum angular velocity during auto.
#define DRIVE_AUTO_MAX_ANG_VEL 3.14_rad_per_s

// The maximum angular acceleration during auto.
#define DRIVE_AUTO_MAX_ANG_ACCEL 3.14_rad_per_s_sq

// The maximum velocity during manual control.
#define DRIVE_MANUAL_MAX_VEL 1.5_mps

// The maximum angular velocity during manual control.
#define DRIVE_MANUAL_MAX_ANG_VEL 360_deg_per_s

// The maximum acceleration during manual control (Must be positive).
#define DRIVE_MANUAL_MAX_ACCEL 3_mps_sq

// The maximum deceleration during manual control (Must be negative).
#define DRIVE_MANUAL_MAX_DECEL -4_mps_sq

// The maximum angular acceleration during manual control (Must be positive).
#define DRIVE_MANUAL_MAX_ANG_ACCEL 6.28_rad_per_s_sq

// The maximum angular deceleration during manual control (Must be negative).
#define DRIVE_MANUAL_MAX_ANG_DECEL -6.28_rad_per_s_sq

// The path where the recorded trajectory is stored.
#define RECORDED_TRAJ_PATH std::filesystem::path("/home/lvuser/recorded_trajectory.csv")

// The path where the motion profile is stored.
#define MOTION_PROFILE_PATH std::filesystem::path("/home/lvuser/trajectory_motion.csv")

// Drivetrain X and Y PID values.
#define DRIVE_XY_P 0.4
#define DRIVE_XY_I 0.0
#define DRIVE_XY_D 0.02

// Drivetrain Theta PID values.
#define DRIVE_THETA_P 4.0
#define DRIVE_THETA_I 0.0
#define DRIVE_THETA_D 0.1

class Drive : public Mechanism {
public:
    Drive(Limelight* limelight, RollingRaspberry* rollingRaspberry);
    ~Drive();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    /**
     * A number of flags that specify different control features
     * of the robot during manual control.
     */
    enum ControlFlag {
        NONE          = 0,
        FIELD_CENTRIC = 1 << 0,
        VICE_GRIP     = 1 << 1,
        BRICK         = 1 << 2,
        RECORDING     = 1 << 3,
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
    void manualControlRelRotation(double xPct, double yPct, double angPct, unsigned flags);

    void manualControlAbsRotation(double xPct, double yPct, units::radian_t angle, unsigned flags);

    /**
     * Runs a trajectory.
     */
    void runTrajectory(const Trajectory& trajectory, const std::map<u_int32_t, Action*>& actionMap);

    /**
     * Returns whether the current process is finished or not.
     */
    bool isFinished() const;

    /**
     * Calibrates the IMU (Pauses the robot for 4 seconds while it
     * calibrates).
     */
    void calibrateIMU();

    /**
     * Returns whether the IMU is calibrated.
     */
    bool isIMUCalibrated();

    /**
     * Resets the position and rotation of the drivetrain on the field
     * to a specified pose.
     */
    void resetOdometry(frc::Pose2d pose = frc::Pose2d());

    /**
     * Returns the position and rotation of the robot on the field.
     */
    frc::Pose2d getEstimatedPose();

    /**
     * Returns the raw rotation of the robot as recorded by the IMU.
     */
    frc::Rotation2d getRotation();

    /**
     * Reloads the recorded trajectory.
     */
    void reloadRecordedTrajectory();

    /**
     * Returns the recorded trajectory.
     */
    inline const Trajectory& getRecordedTrajectory() const { return recordedTrajectory; }

private:
    /**
     * Updates the position and rotation of the drivetrain on the field.
     */
    void updateOdometry();

    /**
     * Executes instructions for when the robot is stopped.
     */
    void execStopped();

    /**
     * Executes the instructions for when the robot is in manual control.
     */
    void execManual();

    /**
     * Executes the instructions for when the robot is running a trajectory.
     */
    void execTrajectory();

    /**
     * Records a state.
     */
    void record_state();

    /**
     * Puts the drivetrain into brick mode (all modules turned towards the center).
     */
    void makeBrick();

    /**
     * Applies the current rotation of the swerve modules as the offset of the
     * magnetic encoders (THIS FUNCTION CAN MESS THINGS UP. USE CAREFULLY).
     */
    void configMagneticEncoders();

    /**
     * Reads the magnetic encoder offsets from the file on the RoboRIO.
     */
    bool readOffsetsFile();

    /**
     * Writes the current magnetic encoder offsets to the file on the RoboRIO.
     */
    void writeOffsetsFile();

    /**
     * Applies the current magnetic encoder offsets to the swerve modules.
     */
    void applyOffsets();

    /**
     * Sets the idle mode of the drive motors.
     */
    void setIdleMode(ThunderCANMotorController::IdleMode mode);

    /**
     * Sets the velocities of the drivetrain.
     */
    void setModuleStates(frc::ChassisSpeeds speeds);
    /**
     * Returns the states of the swerve modules.
     */
    wpi::array<frc::SwerveModuleState, 4> getModuleStates();

    /**
     * Returns the positions of the swerve modules.
     */
    wpi::array<frc::SwerveModulePosition, 4> getModulePositions();

    // Vision camera.
    Limelight* limelight;

    // Raspberry Pi
    RollingRaspberry* rollingRaspberry;

    // The IMU that keeps track of the robot's rotation.
    HardwareManager::DriveIMU imu;

    bool imuCalibrated = false;

    // The locations of the swerve modules on the robot.
    wpi::array<frc::Translation2d, 4> locations {
        frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front left.
        frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back left.
        frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back right.
        frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front right.
    };

    // The swerve modules on the robot.
    wpi::array<SwerveModule*, 4> swerveModules {
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_FL, CAN_SWERVE_ROT_MOTOR_FL, CAN_SWERVE_CAN_CODER_FL, true),
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_BL, CAN_SWERVE_ROT_MOTOR_BL, CAN_SWERVE_CAN_CODER_BL, true),
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_BR, CAN_SWERVE_ROT_MOTOR_BR, CAN_SWERVE_CAN_CODER_BR, true),
      new SwerveModule(CAN_SWERVE_DRIVE_MOTOR_FR, CAN_SWERVE_ROT_MOTOR_FR, CAN_SWERVE_CAN_CODER_FR, false),
    };

    // The magnetic encoder offsets of the swerve modules.
    wpi::array<units::radian_t, 4> offsets { 0_rad, 0_rad, 0_rad, 0_rad };

    /**
     * The helper class that it used to convert chassis speeds into swerve
     * module states.
     */
    frc::SwerveDriveKinematics<4> kinematics { locations };

    /**
     * The class that handles tracking the position of the robot on the
     * field during the match.
     */
    frc::SwerveDrivePoseEstimator<4> poseEstimator {
        kinematics,
        frc::Rotation2d(),
        getModulePositions(),
        frc::Pose2d(),
        { 0.1, 0.1, 0.1 }, // Standard deviations of model states.
        { 0.1, 0.1, 0.1 } // Standard deviations of the vision measurements.
    };

    /**
     * The slew rate limiter to control x and y acceleration and
     * deceleration during manual control.
     * 
     * NOTE: (Waiting for WPILib update to add deceleration)
     */
    frc::SlewRateLimiter<units::meters_per_second> driveRateLimiter { DRIVE_MANUAL_MAX_ACCEL, DRIVE_MANUAL_MAX_DECEL };

    /**
     * The slew rate limiter to control angular acceleration and
     * deceleration during manual control.
     * 
     * NOTE: (Waiting for WPILib update to add deceleration)
     */
    frc::SlewRateLimiter<units::radians_per_second> turnRateLimiter { DRIVE_MANUAL_MAX_ANG_ACCEL, DRIVE_MANUAL_MAX_ANG_DECEL };

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

    // Used to record trajectories when in manual control.
    TrajectoryRecorder trajectoryRecorder;

    frc::Timer teleopTimer;

    units::second_t lastTeleopTime;

    // The recorded trajectory.
    Trajectory recordedTrajectory { RECORDED_TRAJ_PATH };

    // The trajectory that is currently being run.
    const Trajectory* trajectory = nullptr;

    // The available actions.
    const std::map<u_int32_t, Action*>* trajectoryActions = nullptr;

    // Actions that are completed.
    std::vector<u_int32_t> doneTrajectoryActions;

    // The current action.
    std::map<units::second_t, u_int32_t>::const_iterator trajectoryActionIter;

    frc::Timer trajectoryTimer;

    // PID Controller for X and Y axis drivetrain movement.
    frc::PIDController xPIDController { DRIVE_XY_P, DRIVE_XY_I, DRIVE_XY_D },
                       yPIDController { DRIVE_XY_P, DRIVE_XY_I, DRIVE_XY_D };

    // PID Controller for angular drivetrain movement.
    frc::ProfiledPIDController<units::radians> trajectoryThetaPIDController {
        DRIVE_THETA_P, DRIVE_THETA_I, DRIVE_THETA_D,
        frc::TrapezoidProfile<units::radians>::Constraints(DRIVE_AUTO_MAX_ANG_VEL, DRIVE_AUTO_MAX_ANG_ACCEL)
    };
    
    // PID Controller for angular drivetrain movement.
    frc::ProfiledPIDController<units::radians> manualThetaPIDController {
        DRIVE_THETA_P, DRIVE_THETA_I, DRIVE_THETA_D,
        frc::TrapezoidProfile<units::radians>::Constraints(DRIVE_MANUAL_MAX_ANG_VEL, DRIVE_MANUAL_MAX_ANG_ACCEL)
    };

    // The drive controller that will handle the drivetrain movement.
    frc::HolonomicDriveController driveController;

    // Feedback variables.
    frc::ChassisSpeeds chassisSpeeds { 0_mps, 0_mps, 0_rad_per_s };
    frc::Pose2d targetPose;

    // CSV File on the RoboRIO to log drivetrain motion when running a trajectory.
    std::ofstream trajectoryMotionFile { MOTION_PROFILE_PATH };
};
