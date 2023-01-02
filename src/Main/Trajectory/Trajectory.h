#pragma once

#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/math.h>
#include <map>
#include <numbers>
#include <cstdint>
#include <filesystem>

/**
 * Represents a ThunderAuto trajectory for the robot to follow.
 */
class Trajectory {
public:
    /**
     * Represents a single point in a trajectory.
     */
    struct State {
        // The target pose of the robot.
        frc::Pose2d pose;

        // The target velocity of the robot.
        units::meters_per_second_t velocity;
    };

    Trajectory(std::filesystem::path path);
    ~Trajectory();

    /**
     * Samples the trajectory at a specified time.
     */
    State sample(units::second_t time) const;

    /**
     * Returns the duration in seconds of the trajectory.
     */
    units::second_t getDuration() const;

    /**
     * Returns the initial position of the robot.
     */
    frc::Pose2d getInitialPose() const;

    /**
     * Returns the actions with their attributed timestamps.
     */
    inline const std::map<units::second_t, u_int32_t>& getActions() const { return actions; }

private:
    std::map<units::second_t, State> states;
    std::map<units::second_t, u_int32_t> actions;
};
