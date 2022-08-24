#include <Basic/Mechanism.h>
#include <frc/DriverStation.h>

Mechanism::~Mechanism() = default;

void Mechanism::doPersistentConfiguration() { }

void Mechanism::resetToMode(MatchMode mode) { }

void Mechanism::sendFeedback() { }

void Mechanism::process() { }

Mechanism::MatchMode Mechanism::getCurrentMode() {
    if (frc::DriverStation::IsDisabled()) {
        return MatchMode::DISABLED;
    }
    else if (frc::DriverStation::IsAutonomous()) {
        return MatchMode::AUTO;
    } 
    else if (frc::DriverStation::IsTeleop()) {
        return MatchMode::TELEOP;
    }
    else if (frc::DriverStation::IsTest()) {
        return MatchMode::TEST;
    }
    else {
        return MatchMode::DISABLED;
    }
}

Settings Mechanism::settings {};