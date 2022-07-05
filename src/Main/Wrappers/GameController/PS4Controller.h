#pragma once

#include <Wrappers/GameController/GameController.h>
#include <frc/Joystick.h>

class ThunderPS4Controller : public ThunderGameController {
public:
    ThunderPS4Controller(int id);
    ~ThunderPS4Controller();

    bool getTriangleButton();
    bool getCircleButton();
    bool getCrossButton();
    bool getSquareButton();

    inline bool getYButton() { return getTriangleButton(); }
    inline bool getBButton() { return getCircleButton();   }
    inline bool getAButton() { return getCrossButton();    }
    inline bool getXButton() { return getSquareButton();   }

    double getLeftXAxis();
    double getLeftYAxis();
    double getRightXAxis();
    double getRightYAxis();

    bool getLeftBumper();
    bool getRightBumper();
    
    double getLeftTrigger();
    double getRightTrigger();

    bool getShareButton();
    bool getOptionsButton();

    inline bool getBackButton()  { return getShareButton(); }
    inline bool getStartButton() { return getOptionsButton(); }

    bool getLeftStickButton();
    bool getRightStickButton();

    int getDPad();

private:
    frc::Joystick joystick;
};