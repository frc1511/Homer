#include <Basic/Homer.h>
#include <Basic/Mechanism.h>

void Homer::RobotInit() { }

void Homer::RobotPeriodic() {
    for (Mechanism* mech : allMechanisms) {
        mech->sendFeedback();
    }

#ifndef TEST_BOARD
    blinkyBlinky.process();
#endif
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
        puts("*** Persistent configuration activating...");
        
        for (Mechanism* mech : allMechanisms) {
            mech->doPersistentConfiguration();
        }
        
        puts("*** Persistent configuration complete!");
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
