#include <Wrappers/GameController/GameController.h>

ThunderGameController::ThunderGameController(int id)
: controller(id) { }

ThunderGameController::~ThunderGameController() = default;

bool ThunderGameController::getButton(int button) {
    return controller.GetRawButton(button);
}

bool ThunderGameController::getButtonPressed(int button) {
    return controller.GetRawButtonPressed(button);
}

bool ThunderGameController::getButtonReleased(int button) {
    return controller.GetRawButtonReleased(button);
}

double ThunderGameController::getAxis(int axis) {
    return controller.GetRawAxis(axis);
}

int ThunderGameController::getDPad() {
    return controller.GetPOV();
}
