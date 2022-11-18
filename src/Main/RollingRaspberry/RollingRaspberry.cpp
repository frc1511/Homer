#include <RollingRaspberry/RollingRaspberry.h>

RollingRaspberry::RollingRaspberry()
: table(nt::NetworkTableInstance::GetDefault().GetTable(ROLLING_RASP_NT_NAME)) { }

RollingRaspberry::~RollingRaspberry() = default;

bool RollingRaspberry::isConnected() {
    return table->GetBoolean("IsRunning", false);
}

std::optional<frc::Pose2d> RollingRaspberry::getEstimatedRobotPosition() {
    if (table->GetBoolean("HasPose", false)) {
        units::meter_t x = units::meter_t(table->GetNumber("RobotXPosition", 0.0));
        units::meter_t y = units::meter_t(table->GetNumber("RobotYPosition", 0.0));
        units::radian_t rot = units::radian_t(table->GetNumber("RobotRotation", 0.0));

        return frc::Pose2d(x, y, rot);
    }
    else {
        return std::nullopt;
    }
}