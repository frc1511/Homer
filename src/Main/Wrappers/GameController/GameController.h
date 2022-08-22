#pragma once

#include <frc/GenericHID.h>

class ThunderGameController {
public:
    ThunderGameController(int id);
    ~ThunderGameController();

    bool getButton(int button);
    bool getButtonPressed(int button);
    bool getButtonReleased(int button);

    double getAxis(int axis);

    int getDPad();

private:
    frc::GenericHID controller;
};
