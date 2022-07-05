#include <Wrappers/GameController/XBoxController.h>

ThunderXBoxController::ThunderXBoxController(int id)
: joystick(id) { }

ThunderXBoxController::~ThunderXBoxController() = default;

bool ThunderXBoxController::getYButton() {
    return joystick.GetRawButton(4);
}

bool ThunderXBoxController::getBButton() {
    return joystick.GetRawButton(2);
}

bool ThunderXBoxController::getAButton() {
    return joystick.GetRawButton(1);
}

bool ThunderXBoxController::getXButton() {
    return joystick.GetRawButton(3);
}

double ThunderXBoxController::getLeftXAxis() {
    return joystick.GetRawAxis(0);
}

double ThunderXBoxController::getLeftYAxis() {
    return joystick.GetRawAxis(1);
}

double ThunderXBoxController::getRightXAxis() {
    return joystick.GetRawAxis(4);
}

double ThunderXBoxController::getRightYAxis() {
    return joystick.GetRawAxis(5);
}

bool ThunderXBoxController::getLeftBumper() {
    return joystick.GetRawButton(5);
}

bool ThunderXBoxController::getRightBumper() {
    return joystick.GetRawButton(6);
}

double ThunderXBoxController::getLeftTrigger() {
    return joystick.GetRawAxis(2);
}

double ThunderXBoxController::getRightTrigger() {
    return joystick.GetRawAxis(3);
}

bool ThunderXBoxController::getBackButton() {
    return joystick.GetRawButton(7);
}

bool ThunderXBoxController::getStartButton() {
    return joystick.GetRawButton(8);
}

bool ThunderXBoxController::getLeftStickButton() {
    return joystick.GetRawButton(9);
}

bool ThunderXBoxController::getRightStickButton() {
    return joystick.GetRawButton(10);
}

int ThunderXBoxController::getDPad() {
    return joystick.GetPOV();
}