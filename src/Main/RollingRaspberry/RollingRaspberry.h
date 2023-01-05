#pragma once

#include <Basic/Feedback.h>
#include <Basic/Mechanism.h>

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableListener.h>
#include <networktables/StringArrayTopic.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <units/time.h>
#include <map>
#include <mutex>

/**
 * Represents the Raspberry Pi Co-Processor on the Robot.
 */
class RollingRaspberry : public Mechanism {
public:
    RollingRaspberry();
    ~RollingRaspberry();

    void process() override;

    /**
     * Returns the estimated poses of the robot calculated by the Raspberry
     * Pi's vision processing pipeline paired with a FPGA Timestamp.
     */
    std::map<units::second_t, frc::Pose2d> getEstimatedRobotPoses();

private:
    void poseCallback(const nt::Event& event);

    // A network table used to communicate with the r-pi.
    std::shared_ptr<nt::NetworkTable> table;

    bool init = false;

    std::map<units::second_t, frc::Pose2d> poses;

    units::second_t startRobotTimestamp;
    units::second_t startPiTimestamp;

    nt::StringArraySubscriber poseSubscriber;
    NT_Listener poseListener;

    std::mutex posesMutex;
};
