#include <Autonomous/Autonomous.h>
#include <Drive/Drive.h>

Autonomous::Autonomous(Drive* drive)
: drive(drive) {

}

Autonomous::~Autonomous() {

}

void Autonomous::resetToMode(MatchMode mode) {
    delayTimer.Reset();
    delayTimer.Start();
    autoTimer.Reset();
    autoTimer.Start();
    step = 0;
}

void Autonomous::process() {
    // Autonomous delay.
    if (delayTimer.Get().value() <= Feedback::getDouble("thunderdashboard", "auto_start_delay", 0.0)) {
        return;
    }

    switch (currentMode) {
        case DO_NOTHING:
            doNothing();
            break;
        case DRIVE_FORWARD:
            driveForward();
            break;
    }
}

void Autonomous::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
}

void Autonomous::driveForward() {
    if (step == 0) {
        drive->runTrajectory(driveForwardTrajectory);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::sendFeedback() {
    Feedback::sendDouble("Autonomous", "step", step);
    Feedback::sendBoolean("Autonomous", "drive finished", drive->isFinished());

    char buf[256] = "";

    auto handleDashboardString = [&](AutoMode mode, const char* description) {
        // Append mode number to the end of the buffer.
        sprintf(&buf[strlen(buf)], ",%d", mode);

        char mode_str[32];

        // Convert the mode into a string.
        sprintf(mode_str, "%d", mode);
        
        Feedback::sendString("thunderdashboard_auto", mode_str, description);
    };

    handleDashboardString(DO_NOTHING, "Do Nothing?!? Nooooooo!!!!");
    handleDashboardString(DRIVE_FORWARD, "Drive Forward or Something");

    Feedback::sendString("thunderdashboard", "auto_list", buf);
}
