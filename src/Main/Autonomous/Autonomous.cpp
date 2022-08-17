#include <Autonomous/Autonomous.h>
#include <Drive/Drive.h>

Autonomous::Autonomous(Drive* drive)
: drive(drive) { }

Autonomous::~Autonomous() = default;

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
        case AutoMode::DEMO_GRATEFUL_RED:
            gratefulRed();
            break;
        case AutoMode::GREAT_G3_ADVENTURE_AWAY:
            greatG3AdventureAway();
            break;
        case AutoMode::GREAT_G3_ADVENTURE_BACK:
            greatG3AdventureBack();
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
        drive->runTrajectory(lineTrajectory, actions);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::greatHallwayAdventure() {
    if (step == 0) {
        drive->runTrajectory(greatHallwayAdventureTrajectory, actions);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::gratefulRed() {
    if (step == 0) {
        drive->runTrajectory(gratefulRedTrajectory, actions);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::greatG3AdventureAway() {
    if (step == 0) {
        drive->runTrajectory(greatG3AdventureAwayTrajectory, actions);
        ++step;
    }
    else if (step == 1 && drive->isFinished()) {
        ++step;
    }
}

void Autonomous::greatG3AdventureBack() {
    if (step == 0) {
        drive->runTrajectory(greatG3AdventureBackTrajectory, actions);
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
    sendAutoMode(AutoMode::DEMO_GRATEFUL_RED, "Grateful Red Demo Path");
    sendAutoMode(AutoMode::GREAT_G3_ADVENTURE_AWAY, "Leave G3");
    sendAutoMode(AutoMode::GREAT_G3_ADVENTURE_BACK, "Enter G3");

    Feedback::sendString("thunderdashboard", "auto_list", buffer);
}

Autonomous::PauseAction::PauseAction(units::second_t dur)
: duration(dur) {
    timer.Reset();
    timer.Stop();
}

Autonomous::PauseAction::~PauseAction() = default;

Action::Result Autonomous::PauseAction::process() {
    timer.Start();

    if (timer.Get() >= duration) {
        timer.Reset();
        timer.Stop();
        return Result::DONE;
    }

    return Result::WORKING;
}

Autonomous::MessageAction::MessageAction(const char* _msg)
: msg(_msg) { }

Autonomous::MessageAction::~MessageAction() = default;

Action::Result Autonomous::MessageAction::process() {
    std::cout << msg;
    return Action::Result::DONE;
}
