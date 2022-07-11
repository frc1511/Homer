#include <Control/Controls.h>
#include <wpi/numbers>
#include <cmath>

#define AXIS_DEADZONE 0.1

Controls::Controls(Drive* drive)
: drive(drive) {

}

Controls::~Controls() {

}

void Controls::process() {
    doDrive();
    doAux();
    doSwitchPanel();
}

void Controls::doDrive() {
    bool brickDrive = driveController.getCrossButton();
    bool viceGrip = driveController.getCircleButton();
    bool toggleCamera = driveController.getSquareButton();

    double xVel = driveController.getLeftXAxis();
    double yVel = driveController.getLeftYAxis();
    double angVel = driveController.getRightXAxis();

    bool zeroRotation = driveController.getOptionsButton();
    bool calGyro = driveController.getShareButton();

    unsigned ctrlFlags = Drive::ControlFlags::NONE;

    if (driveFieldCentric) {
        ctrlFlags |= Drive::ControlFlags::FIELD_CENTRIC;
    }

    if (brickDrive) {
        ctrlFlags |= Drive::ControlFlags::BRICK;
    }

    if (viceGrip) {
        ctrlFlags |= Drive::ControlFlags::VICE_GRIP;
    }

    static bool whichCamera = false;
    if (toggleCamera) {
        whichCamera = !whichCamera;
    }

    if (zeroRotation) {
        drive->zeroRotation();
    }

    if (calGyro) {
        drive->calibrateIMU();
        drive->zeroRotation();
    }

    double finalXVel = 0.0;
    double finalYVel = 0.0;
    double finalAngVel = 0.0;

    // Improves the joystick axis to be smoother and easier to control.
    auto improveAxis = [](double axis) -> double {
        return std::sin(axis * (wpi::numbers::pi / 2.0));
    };

    if (std::fabs(xVel) > AXIS_DEADZONE) {
        finalXVel = improveAxis(xVel);
    }
    if (std::fabs(yVel) > AXIS_DEADZONE) {
        finalYVel = improveAxis(yVel);
    }
    if (std::fabs(angVel) > AXIS_DEADZONE) {
        finalAngVel = improveAxis(angVel);
    }

    // Control the drivetrain.
    drive->manualControl(finalXVel, -finalYVel, -finalAngVel, ctrlFlags);
}

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();
    return settings.isCraterMode
        && driveController.getTriangleButton() && driveController.getDPad() == 180
        && auxController.getCrossButton() && auxController.getDPad() == 0;
}

void Controls::doAux() {
    // D:
}

void Controls::doSwitchPanel() {
    settings.isCraterMode = switchPanel.GetRawButton(1);
    driveFieldCentric = switchPanel.GetRawButton(2);
}

void Controls::sendFeedback() {

}