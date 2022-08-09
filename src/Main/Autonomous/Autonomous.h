#pragma once

#include <Basic/Mechanism.h>
#include <Basic/Feedback.h>
#include <frc/Timer.h>
#include <Drive/Trajectory.h>

#define DEPLOY_DIR "/home/lvuser/deploy/"

class Drive;

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive);
    ~Autonomous();

    void process() override;
    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;

private:
    enum class AutoMode {
        DO_NOTHING = 0, // Do nothing or something.
        LINE = 1, // Drive straight 6 meters and drive back.
        GREAT_HALLWAY_ADVENTURE = 2, // Drive around the hallway.
    };

    void doNothing();
    void line();
    void greatHallwayAdventure();

    AutoMode currentMode = AutoMode::DO_NOTHING;

    frc::Timer delayTimer,
               autoTimer;

    std::size_t step = 0;

    Drive* drive;

    Trajectory lineTrajectory { DEPLOY_DIR "line/line.csv" };
    Trajectory greatHallwayAdventureTrajectory { DEPLOY_DIR "great_hallway_adventure/great_hallway_adventure.csv" };
};
