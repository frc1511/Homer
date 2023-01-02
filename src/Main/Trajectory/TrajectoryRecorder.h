#pragma once

#include <Trajectory/Trajectory.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <map>
#include <algorithm>
#include <filesystem>

/**
 * Call it AutoForPeter :D
 */
class TrajectoryRecorder {
public:
    TrajectoryRecorder();
    ~TrajectoryRecorder();

    /**
     * Clears the recorded data.
     */
    void clear();

    /**
     * Outputs the current trajectory to a CSV file on the RoboRIO.
     */
    void writeToCSV(std::filesystem::path path);

    /**
     * Adds a state to the current trajectory being recorded.
     */
    void addState(units::second_t deltaTime, frc::Pose2d currentPose);

private:
   std::map<units::second_t, Trajectory::State> states;

   decltype(states)::const_iterator lastStateIt;
};
