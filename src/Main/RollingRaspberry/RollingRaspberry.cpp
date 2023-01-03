#include <RollingRaspberry/RollingRaspberry.h>
#include <Util/Parser.h>

#define ROLLING_RASP_NT_NAME "RollingRaspberry"

RollingRaspberry::RollingRaspberry()
: table(nt::NetworkTableInstance::GetDefault().GetTable(ROLLING_RASP_NT_NAME)) { }

RollingRaspberry::~RollingRaspberry() = default;

void RollingRaspberry::process() {
    bool running = table->GetBoolean("IsRunning", false);

    if (!connected && running) {
        startRobotTimestamp = frc::Timer::GetFPGATimestamp();
        startPiTimestamp = units::second_t(table->GetNumber("Uptime", 0.0));
        lastPiTimestamp = startPiTimestamp;
    }
     
    connected = running;
}

bool RollingRaspberry::isConnected() {
    return connected;
}

std::pair<units::second_t, std::vector<frc::Pose2d>> RollingRaspberry::getEstimatedRobotPoses() {
    std::vector<std::string> poseStrs = table->GetStringArray("Poses", {});
    units::second_t poseTime(table->GetNumber("PoseTime", 0.0));

    if (!connected || poseTime == lastPiTimestamp) {
        return std::make_pair(0_s, std::vector<frc::Pose2d>());
    }

    lastPiTimestamp = poseTime;

    std::vector<frc::Pose2d> poses;
    for (const std::string& poseStr : poseStrs) {
        Parser::Iter currIt = poseStr.cbegin();

        units::meter_t x(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::meter_t y(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::radian_t rot(Parser::parseNumber(currIt, poseStr.cend()));

        poses.emplace_back(x, y, rot);
    }

    units::second_t timeSinceStart = poseTime - startPiTimestamp;

    return std::make_pair(startRobotTimestamp + timeSinceStart, poses);
}
