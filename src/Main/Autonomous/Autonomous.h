#pragma once

#include <Basic/Mechanism.h>
#include <Basic/Feedback.h>
#include <frc/Timer.h>
#include <Trajectory/Trajectory.h>
#include <Autonomous/Action.h>
#include <map>
#include <cstdint>

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
        DEMO_LONG = 3,
    };

    void doNothing();
    void line();
    void greatHallwayAdventure();
    void demoLong();

    AutoMode currentMode = AutoMode::DO_NOTHING;

    frc::Timer delayTimer,
               autoTimer;

    std::size_t step = 0;

    Drive* drive;

    Trajectory lineTrajectory                  { DEPLOY_DIR "line/line.csv" };
    Trajectory greatHallwayAdventureTrajectory { DEPLOY_DIR "great_hallway_adventure/great_hallway_adventure.csv" };
    Trajectory demoLongTrajectory              { DEPLOY_DIR "demo/long.csv" };

    /**
     * An action that pauses the path for a specified number of seconds.
     */
    class PauseAction : public Action {
    public:
        PauseAction(units::second_t duration);
        ~PauseAction();

        Result process() override;

    private:
        frc::Timer timer;
        units::second_t duration;
    };
    
    /**
     * The action keys (should match the bits set in the trajectory CSV file).
     */
    enum ActionID : u_int32_t {
        WAIT_A_BIT        = 1 << 0,
        WAIT_A_BIT_LONGER = 1 << 1,
    };

    PauseAction pause3sAction { 3_s };
    PauseAction pause5sAction { 5_s };

    /**
     * A map of the actions that are available to each autonomous mode.
     */
    std::map<u_int32_t, Action*> actions {
        { ActionID::WAIT_A_BIT,        &pause3sAction },
        { ActionID::WAIT_A_BIT_LONGER, &pause5sAction },
    };
};
