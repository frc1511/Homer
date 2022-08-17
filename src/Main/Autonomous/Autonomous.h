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
        DEMO_GRATEFUL_RED = 3,
        GREAT_G3_ADVENTURE_AWAY = 4,
        GREAT_G3_ADVENTURE_BACK = 5,
    };

    void doNothing();
    void line();
    void greatHallwayAdventure();
    void gratefulRed();
    void greatG3AdventureAway();
    void greatG3AdventureBack();

    AutoMode currentMode = AutoMode::DO_NOTHING;

    frc::Timer delayTimer,
               autoTimer;

    std::size_t step = 0;

    Drive* drive;

    Trajectory lineTrajectory                  { DEPLOY_DIR "line/line.csv" };
    Trajectory greatHallwayAdventureTrajectory { DEPLOY_DIR "great_hallway_adventure/great_hallway_adventure.csv" };
    Trajectory gratefulRedTrajectory           { DEPLOY_DIR "demo/GratefulRed.csv" };
    Trajectory greatG3AdventureAwayTrajectory  { DEPLOY_DIR "great_g3_adventure/great_g3_adventure_away.csv" };
    Trajectory greatG3AdventureBackTrajectory  { DEPLOY_DIR "great_g3_adventure/great_g3_adventure_back.csv" };

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

    class MessageAction : public Action {
    public:
        MessageAction(const char* msg = "Hello, Ishan!\n");
        ~MessageAction();

        Result process() override;

    private:
        const char* msg;
    };

    PauseAction pause3sAction { 3_s };
    MessageAction msgAction;

    /**
     * A map of the actions that are available to each autonomous mode.
     */
    std::map<u_int32_t, Action*> actions {
        { 1 << 0, &pause3sAction },
        { 1 << 1, &msgAction },
    };
};
