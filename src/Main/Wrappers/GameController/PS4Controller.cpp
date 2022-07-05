#include <Wrappers/GameController/PS4Controller.h>

ThunderPS4Controller::ThunderPS4Controller(int id)
: joystick(id) { }

ThunderPS4Controller::~ThunderPS4Controller() = default;

bool ThunderPS4Controller::getTriangleButton() {
    return joystick.GetRawButton(4);
}

bool ThunderPS4Controller::getCircleButton() {
    return joystick.GetRawButton(3);
}

bool ThunderPS4Controller::getCrossButton() {
    return joystick.GetRawButton(2);
}

bool ThunderPS4Controller::getSquareButton() {
    return joystick.GetRawButton(1);
}

double ThunderPS4Controller::getLeftXAxis() {
    return joystick.GetRawAxis(0);
}

double ThunderPS4Controller::getLeftYAxis() {
    return joystick.GetRawAxis(1);
}

double ThunderPS4Controller::getRightXAxis() {
    return joystick.GetRawAxis(2);
}

double ThunderPS4Controller::getRightYAxis() {
    return joystick.GetRawAxis(5);
}

bool ThunderPS4Controller::getLeftBumper() {
    return joystick.GetRawButton(5);
}

bool ThunderPS4Controller::getRightBumper() {
    return joystick.GetRawButton(6);
}

double ThunderPS4Controller::getLeftTrigger() {
    return joystick.GetRawAxis(3);
}

double ThunderPS4Controller::getRightTrigger() {
    return joystick.GetRawAxis(4);
}

bool ThunderPS4Controller::getShareButton() {
    return joystick.GetRawButton(9);
}

bool ThunderPS4Controller::getOptionsButton() {
    return joystick.GetRawButton(10);
}

bool ThunderPS4Controller::getLeftStickButton() {
    return joystick.GetRawButton(11);
}

bool ThunderPS4Controller::getRightStickButton() {
    return joystick.GetRawButton(12);
}

int ThunderPS4Controller::getDPad() {
    return joystick.GetPOV();
}