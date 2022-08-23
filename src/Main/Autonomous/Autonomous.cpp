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
            runTrajectory(lineTrajectory);
            break;
        case AutoMode::GREAT_HALLWAY_ADVENTURE:
            runTrajectory(greatHallwayAdventureTrajectory);
            break;
        case AutoMode::DEMO_GRATEFUL_RED:
            runTrajectory(gratefulRedTrajectory);
            break;
        case AutoMode::GREAT_G3_ADVENTURE_TO_HALLWAY:
            runTrajectory(greatG3AdventureAwayTrajectory);
            break;
        case AutoMode::GREAT_G3_ADVENTURE_TO_SHOP:
            runTrajectory(greatG3AdventureBackTrajectory);
            break;
        case AutoMode::RECORDED:
            runTrajectory(drive->getRecordedTrajectory());
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

void Autonomous::runTrajectory(const Trajectory& trajectory) {
    if (step == 0) {
        drive->runTrajectory(trajectory, actions);
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

    sendAutoMode(AutoMode::DO_NOTHING, "Do Nothing");
    sendAutoMode(AutoMode::LINE, "Line");
    sendAutoMode(AutoMode::GREAT_HALLWAY_ADVENTURE, "Great Hallway Adventure");
    sendAutoMode(AutoMode::DEMO_GRATEFUL_RED, "Grateful Red Demo");
    sendAutoMode(AutoMode::GREAT_G3_ADVENTURE_TO_HALLWAY, "Leave G3");
    sendAutoMode(AutoMode::GREAT_G3_ADVENTURE_TO_SHOP, "Enter G3");
    sendAutoMode(AutoMode::RECORDED, "Recorded Trajectory");

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
