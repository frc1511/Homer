#pragma once

#include <Basic/Feedback.h>
#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <vector>

/**
 * Represents the Raspberry Pi Co-Processor on the Robot.
 */
class RollingRaspberry : public Mechanism {
public:
    RollingRaspberry();
    ~RollingRaspberry();

    /**
     * Returns whether the Raspberry Pi is connected and the robot program is
     * running.
     */
    bool isConnected();

    /**
     * Returns the estimated poses of the robot calculated by the Raspberry
     * Pi's vision processing pipeline.
     */
    std::vector<frc::Pose2d> getEstimatedRobotPoses();

private:
    // A network table used to communicate with the r-pi.
    std::shared_ptr<nt::NetworkTable> table;
};
