#pragma once

class ThunderGameController {
public:
    virtual ~ThunderGameController() = default;

    virtual bool getTriangleButton() = 0;
    virtual bool getCircleButton() = 0;
    virtual bool getCrossButton() = 0;
    virtual bool getSquareButton() = 0;

    virtual bool getYButton() = 0;
    virtual bool getBButton() = 0;
    virtual bool getAButton() = 0;
    virtual bool getXButton() = 0;

    virtual double getLeftXAxis() = 0;
    virtual double getLeftYAxis() = 0;
    virtual double getRightXAxis() = 0;
    virtual double getRightYAxis() = 0;

    virtual bool getLeftBumper() = 0;
    virtual bool getRightBumper() = 0;
    
    virtual double getLeftTrigger() = 0;
    virtual double getRightTrigger() = 0;

    virtual bool getShareButton() = 0;
    virtual bool getOptionsButton() = 0;

    virtual bool getBackButton() = 0;
    virtual bool getStartButton() = 0;

    virtual bool getLeftStickButton() = 0;
    virtual bool getRightStickButton() = 0;

    virtual int getDPad() = 0;
};