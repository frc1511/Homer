#include <Autonomous/Autonomous.h>
#include <Drive/Drive.h>

Autonomous::Autonomous(Drive* drive)
: drive(drive) { }

Autonomous::~Autonomous() { }

void Autonomous::resetToMode(MatchMode mode) {
    delayTimer.Reset();
    delayTimer.Start();
    autoTimer.Reset();
    autoTimer.Start();
    step = 0;
}

void Autonomous::process() {
    currentMode = static_cast<AutoMode>(Feedback::getDouble("Auto", "Mode", 0));

    // Autonomous delay.
    if (delayTimer.Get().value() <= Feedback::getDouble("thunderdashboard", "auto_start_delay", 0.0)) {
        return;
    }

    switch (currentMode) {
        case AutoMode::DO_NOTHING:
            doNothing();
            break;
        case AutoMode::LINE:
            line();
            break;
        case AutoMode::GREAT_HALLWAY_ADVENTURE:
            greatHallwayAdventure();
            break;
        case AutoMode::DEMO_LONG:
            demoLong();
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

void Autonomous::line() {
    if (step == 0) {
        drive->runTrajectory(lineTrajectory);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::greatHallwayAdventure() {
    if (step == 0) {
        drive->runTrajectory(greatHallwayAdventureTrajectory);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::demoLong() {
    if (step == 0) {
        drive->runTrajectory(demoLongTrajectory);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::sendFeedback() {
    Feedback::sendDouble("Autonomous", "step", step);
    Feedback::sendBoolean("Autonomous", "drive finished", drive->isFinished());

    char buffer[256] = "";

    auto sendAutoMode = [&](AutoMode mode, const char* description) {
        // Append mode number to the end of the buffer.
        sprintf(&buffer[strlen(buffer)], ",%d", static_cast<int>(mode));

        char mode_str[32];

        // Convert the mode into a string.
        sprintf(mode_str, "%d", static_cast<int>(mode));
        
        Feedback::sendString("thunderdashboard_auto", mode_str, description);
    };

    sendAutoMode(AutoMode::DO_NOTHING, "Do Nothing?!? Nooooooo!!!!");
    sendAutoMode(AutoMode::LINE, "Drive forward 6m and drive back while turning");
    sendAutoMode(AutoMode::GREAT_HALLWAY_ADVENTURE, "Drive Forward or Something");
    sendAutoMode(AutoMode::DEMO_LONG, "Demo Long");

    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}
