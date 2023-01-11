#include <RollingRaspberry/RollingRaspberry.h>
#include <Util/Parser.h>
#include <chrono>

RollingRaspberry::RollingRaspberry()
: table(nt::NetworkTableInstance::GetDefault().GetTable("RollingRaspberry")), // Rolling Raspberry network table.
  posesSubtable(table->GetSubTable("Poses")), // Poses sub-table.
  poseListener(posesSubtable->AddListener( // Pose listener.
    nt::EventFlags::kValueAll,
    [this](nt::NetworkTable*, std::string_view key, const nt::Event& event) {
        if (event.GetValueEventData()->value.type() == NT_Type::NT_STRING) {
            this->poseCallback(key, event);
        }
    }
  )) {

}

RollingRaspberry::~RollingRaspberry() = default;

void RollingRaspberry::process() { }

void RollingRaspberry::poseCallback(std::string_view key, const nt::Event& event) {
    decltype(poses) newPoses;
    const std::string poseStr(event.GetValueEventData()->value.GetString());
    Parser::Iter currIt(poseStr.cbegin());

    // Parse each pose estimation string, format: "xPos,yPos,rotation"
    units::second_t timestamp(std::atoi(key.data()));

    // 0 means error, unix timestamp won't roll over until 10:14 PM on 1/7/2038 so we don't have to worry about it for a while.
    if (!timestamp) return;
    units::meter_t xPos(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
    units::meter_t yPos(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
    units::radian_t rotation(Parser::parseNumber(currIt, poseStr.cend()));

    // Current timepoint.
    const auto timePoint = std::chrono::system_clock::now();
    // Unix timestamp.
    units::second_t unixTimestamp(std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count() * 0.001);

    // Calculate time since estimate.
    units::second_t timeSinceEstimate = unixTimestamp - timestamp;

    // Robot FPGA timestamp (uptime).
    units::second_t fpgaTimestamp = frc::Timer::GetFPGATimestamp();
    
    // Add new pose.
    newPoses.emplace(fpgaTimestamp - timeSinceEstimate, frc::Pose2d(xPos, yPos, rotation));

    // Save the new pose estimations.
    if (!newPoses.empty()) {
        std::lock_guard lk(posesMutex);

        poses.insert(newPoses.cbegin(), newPoses.cend());
    }
}

std::map<units::second_t, frc::Pose2d> RollingRaspberry::getEstimatedRobotPoses() {
    std::lock_guard lk(posesMutex);

    // Get the current pose estimations.
    auto currPoses = poses;

    // Clear the pose estimations (don't need them anymore).
    poses.clear();

    return currPoses;
}
