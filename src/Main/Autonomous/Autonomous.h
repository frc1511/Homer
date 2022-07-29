#pragma once

#include <Basic/Mechanism.h>
#include <Basic/Feedback.h>
#include <frc/Timer.h>
#include <Drive/Trajectory.h>

class Drive;

class Autonomous : public Mechanism {
public:
    Autonomous(Drive* drive);
    ~Autonomous();

    void process() override;
    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;

private:
    enum AutoMode {
        DO_NOTHING = 0, // Do nothing or something.
        DRIVE_FORWARD = 1,
    };

    void doNothing();
    void driveForward();

    AutoMode currentMode = DO_NOTHING;

    frc::Timer delayTimer,
               autoTimer;

    std::size_t step = 0;

    Drive* drive;

    Trajectory driveForwardTrajectory { "drive_forward.csv" };
};