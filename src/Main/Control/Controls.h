#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <Drive/Drive.h>
#include <Hardware/IOMap.h>
#include <Illumination/BlinkyBlinky.h>

class Controls : public Mechanism {
public:
    Controls(Drive* drive, BlinkyBlinky* blinkyBlinky = nullptr);
    ~Controls();
    
    void process() override;
    void sendFeedback() override;

    bool getShouldPersistConfig();
    
private:
    ThunderGameController driveController {0};
    ThunderGameController auxController {1};
    ThunderGameController switchPanel {2};

    void doDrive();
    void doAux();
    void doSwitchPanel();

    bool driveRobotCentric = false;
    unsigned driveCtrlFlags = Drive::ControlFlag::NONE;

    bool driveRecording = false;

    Drive* drive;
    BlinkyBlinky* blinkyBlinky;
};
