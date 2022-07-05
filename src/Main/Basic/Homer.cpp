#include <Basic/Homer.h>
#include <Basic/Mechanism.h>

void Homer::RobotInit() { }

void Homer::RobotPeriodic() {
    for (Mechanism* mech : allMechanisms) {
        mech->sendFeedback();
    }
}

void Homer::AutonomousInit() {
    reset(Mechanism::MatchMode::AUTO);
}

void Homer::AutonomousPeriodic() {
    drive.process();
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

void Homer::DisabledPeriodic() { }

void Homer::TestInit() {
    if (controls.getShouldPersistConfig()) {
        puts("\033[34\033[1m*** Persistent configuration activating...\033[0m");
        
        for (Mechanism* mech : allMechanisms) {
            mech->doPersistentConfiguration();
        }
        
        puts("\033[34\033[1m*** Persistent configuration complete!\033[0m");
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
