#pragma once

#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <wpi/numbers>
#include <units/math.h>
#include <map>

/**
 * Represents a ThunderAuto trajectory for the robot to follow.
 */
class Trajectory {
public:
    /**
     * Represents a single point in a trajectory.
     */
    struct State {
        // The target x position of the robot.
        units::meter_t xPos;
        // The target y position of the robot.
        units::meter_t yPos;

        // The target velocity of the robot.
        units::meters_per_second_t velocity;

        // The target rotation of the robot.
        frc::Rotation2d rotation;
    };

    Trajectory(const char* path);
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
    std::map<units::second_t, unsigned> getActions() const;

private:
    std::map<units::second_t, State> states;
    std::map<units::second_t, unsigned> actions;
};
