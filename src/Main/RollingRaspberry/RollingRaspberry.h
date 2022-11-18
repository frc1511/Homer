#pragma once

#include <Basic/Feedback.h>
#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <optional>

#define ROLLING_RASP_NT_NAME "RollingRaspberry"

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
     * Returns the estimated position of the robot calculated by the Raspberry
     * Pi's vision processing pipeline.
     * 
     * Will return std::nullopt when the vision processing pipeline fails to
     * identify any targets, or when the raspberry pi is disconnected/not
     * running.
    */
    std::optional<frc::Pose2d> getEstimatedRobotPosition();

private:
    // A network table used to communicate with the r-pi.
    std::shared_ptr<nt::NetworkTable> table;
};
