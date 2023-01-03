#pragma once

#include <Basic/Feedback.h>
#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <vector>
#include <units/time.h>

/**
 * Represents the Raspberry Pi Co-Processor on the Robot.
 */
class RollingRaspberry : public Mechanism {
public:
    RollingRaspberry();
    ~RollingRaspberry();

    void process() override;

    /**
     * Returns whether the Raspberry Pi is connected and the robot program is
     * running.
     */
    bool isConnected();

    /**
     * Returns the estimated poses of the robot calculated by the Raspberry
     * Pi's vision processing pipeline paired with a FPGA Timestamp.
     */
    std::pair<units::second_t, std::vector<frc::Pose2d>> getEstimatedRobotPoses();

private:
    // A network table used to communicate with the r-pi.
    std::shared_ptr<nt::NetworkTable> table;

    bool connected = false;

    units::second_t startRobotTimestamp;
    units::second_t startPiTimestamp;

    units::second_t lastPiTimestamp;
};
