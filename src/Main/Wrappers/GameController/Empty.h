#pragma once

#include <Wrappers/GameController/GameController.h>

class ThunderGameControllerEmpty : public ThunderGameController {
public:
    inline ThunderGameControllerEmpty(int id) { }
    ~ThunderGameControllerEmpty() = default;

    inline bool getTriangleButton() { return false; }
    inline bool getCircleButton()   { return false; }
    inline bool getCrossButton()    { return false; }
    inline bool getSquareButton()   { return false; }

    inline bool getYButton() { return false; }
    inline bool getBButton() { return false; }
    inline bool getAButton() { return false; }
    inline bool getXButton() { return false; }

    inline double getLeftXAxis()  { return 0.0; }
    inline double getLeftYAxis()  { return 0.0; }
    inline double getRightXAxis() { return 0.0; }
    inline double getRightYAxis() { return 0.0; }

    inline bool getLeftBumper()  { return false; }
    inline bool getRightBumper() { return false; }

    inline double getLeftTrigger()  { return 0.0; }
    inline double getRightTrigger() { return 0.0; }

    inline bool getShareButton()   { return false; }
    inline bool getOptionsButton() { return false; }

    inline bool getBackButton()  { return false; }
    inline bool getStartButton() { return false; }

    inline bool getLeftStickButton()  { return false; }
    inline bool getRightStickButton() { return false; }

    inline int getDPad() { return 0; }
};