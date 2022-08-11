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
    HardwareManager::DriveGameController driveController {0};
    HardwareManager::AuxGameController auxController {1};
    frc::Joystick switchPanel {2};

    void doDrive();
    void doAux();
    void doSwitchPanel();

    bool driveRobotCentric = false;
    unsigned driveCtrlFlags = Drive::ControlFlag::NONE;

    Drive* drive;
    BlinkyBlinky* blinkyBlinky;
};