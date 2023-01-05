#include <RollingRaspberry/RollingRaspberry.h>
#include <Util/Parser.h>

RollingRaspberry::RollingRaspberry()
: table(nt::NetworkTableInstance::GetDefault().GetTable("RollingRaspberry")),
  poseSubscriber(table->GetStringArrayTopic("Poses").Subscribe({})),
  poseListener(nt::NetworkTableInstance::GetDefault().AddListener(
    poseSubscriber,
    nt::EventFlags::kValueAll,
    [this](const nt::Event& event) {
        this->poseCallback(event);
    }
  )) { }

RollingRaspberry::~RollingRaspberry() = default;

void RollingRaspberry::process() {
    bool running = table->GetBoolean("IsRunning", false);

    if (!init && running) {
        startRobotTimestamp = frc::Timer::GetFPGATimestamp();
        startPiTimestamp = units::second_t(table->GetNumber("Uptime", 0.0));
        init = true;
    }
}

void RollingRaspberry::poseCallback(const nt::Event& event) {
    // Shouldn't happen, but the guard is here anyways.
    if (!init) return;

    decltype(poses) newPoses;
    const auto& poseStrs = event.GetValueEventData()->value.GetStringArray();

    // Parse each pose estimation string, format: "timestamp,xPos,yPos,rotation"
    for (const std::string& poseStr : poseStrs) {
        Parser::Iter currIt = poseStr.cbegin();

        units::second_t timestamp(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::meter_t xPos(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::meter_t yPos(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::radian_t rotation(Parser::parseNumber(currIt, poseStr.cend()));

        units::second_t timeSinceStart = timestamp - startPiTimestamp;
        
        newPoses.emplace(startRobotTimestamp + timeSinceStart, frc::Pose2d(xPos, yPos, rotation));
    }

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
