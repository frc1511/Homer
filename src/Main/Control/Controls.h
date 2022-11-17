#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <Drive/Drive.h>
#include <Hardware/IOMap.h>
#include <Illumination/BlinkyBlinky.h>
#include <Vision/Limelight.h>
#include <numbers>

class Controls : public Mechanism {
public:
    Controls(Drive* drive, Limelight* limelight, BlinkyBlinky* blinkyBlinky = nullptr);
    ~Controls();
    
    void process() override;
    void processInDisabled();
    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;

    bool getShouldPersistConfig();
    
private:
    ThunderGameController driveController {0};
    ThunderGameController auxController {1};
    ThunderGameController switchPanel {2};

    void doDrive();
    void doAux();
    void doSwitchPanel();

    bool whichCamera = false;

    bool driveRobotCentric = false;
    unsigned driveCtrlFlags = Drive::ControlFlag::NONE;

    bool driveRecording = false;
    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;

    Drive* drive;
    Limelight* limelight;
    BlinkyBlinky* blinkyBlinky;
};
