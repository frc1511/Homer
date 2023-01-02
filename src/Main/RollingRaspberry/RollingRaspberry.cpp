#include <RollingRaspberry/RollingRaspberry.h>
#include <Util/Parser.h>

#define ROLLING_RASP_NT_NAME "RollingRaspberry"

RollingRaspberry::RollingRaspberry()
: table(nt::NetworkTableInstance::GetDefault().GetTable(ROLLING_RASP_NT_NAME)) { }

RollingRaspberry::~RollingRaspberry() = default;

bool RollingRaspberry::isConnected() {
    return table->GetBoolean("IsRunning", false);
}

std::vector<frc::Pose2d> RollingRaspberry::getEstimatedRobotPoses() {
    std::vector<std::string> poseStrs = table->GetStringArray("Poses", {});

    std::vector<frc::Pose2d> poses;
    for (const std::string& poseStr : poseStrs) {
        Parser::Iter currIt = poseStr.cbegin();

        units::meter_t x(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::meter_t y(Parser::parseNumber(currIt, poseStr.cend())); ++currIt;
        units::radian_t rot(Parser::parseNumber(currIt, poseStr.cend()));

        poses.emplace_back(x, y, rot);
    }

    return poses;
}
