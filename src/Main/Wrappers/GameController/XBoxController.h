#pragma once

#include <Wrappers/GameController/GameController.h>
#include <frc/Joystick.h>

class ThunderXBoxController : public ThunderGameController {
public:
    ThunderXBoxController(int id);
    ~ThunderXBoxController();

    inline bool getTriangleButton() { return getYButton(); }
    inline bool getCircleButton()   { return getBButton(); }
    inline bool getCrossButton()    { return getAButton(); }
    inline bool getSquareButton()   { return getXButton(); }

    bool getYButton();
    bool getBButton();
    bool getAButton();
    bool getXButton();

    double getLeftXAxis();
    double getLeftYAxis();
    double getRightXAxis();
    double getRightYAxis();

    bool getLeftBumper();
    bool getRightBumper();
    
    double getLeftTrigger();
    double getRightTrigger();

    inline bool getShareButton()   { return getBackButton(); }
    inline bool getOptionsButton() { return getStartButton(); }

    bool getBackButton();
    bool getStartButton();

    bool getLeftStickButton();
    bool getRightStickButton();

    int getDPad();

private:
    frc::Joystick joystick;
};