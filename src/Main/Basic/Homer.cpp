#include <Basic/Homer.h>
#include <Basic/Mechanism.h>
#include <fmt/core.h>

using namespace frc;

void Homer::RobotInit() { }

void Homer::RobotPeriodic() {
    for (Mechanism* mech : allMechanisms) {
        mech->sendFeedback();
    }

    for (Mechanism* mech : universalMechanisms) {
        mech->process();
    }
}

void Homer::AutonomousInit() {
    reset(Mechanism::MatchMode::AUTO);
}

void Homer::AutonomousPeriodic() {
    drive.process();
    autonomous.process();
}

void Homer::TeleopInit() {
    reset(Mechanism::MatchMode::TELEOP);
}

void Homer::TeleopPeriodic() {
    drive.process();
    controls.process();
}

void Homer::DisabledInit() {
    reset(Mechanism::MatchMode::DISABLED);
}

void Homer::DisabledPeriodic() {
    controls.processInDisabled();
}

void Homer::TestInit() {
    if (controls.getShouldPersistConfig()) {
        fmt::print("*** Persistent configuration activating...\n");
        for (Mechanism* mech : allMechanisms) {
          mech->doPersistentConfiguration();
        }
        fmt::print("*** Persistent configuration complete!\n");
    }
    reset(Mechanism::MatchMode::TEST);
}

void Homer::TestPeriodic() { }

void Homer::reset(Mechanism::MatchMode mode) {
    for (Mechanism* mech : allMechanisms) {
        mech->resetToMode(mode);
    }
}

int main() {
    return frc::StartRobot<Homer>();
}
