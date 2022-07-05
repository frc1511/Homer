#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/HardwareManager.h>
#include <Drive/Drive.h>

class Controls : public Mechanism {
public:
    Controls(Drive* drive);
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

    bool driveFieldCentric = false;

    Drive* drive;
};