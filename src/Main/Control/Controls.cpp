#include <Control/Controls.h>
#include <wpi/numbers>
#include <cmath>

#define AXIS_DEADZONE 0.1

Controls::Controls(Drive* drive, BlinkyBlinky* blinkyBlinky)
: drive(drive), blinkyBlinky(blinkyBlinky) { }

Controls::~Controls() { }

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

    bool wasBrickDrive = driveCtrlFlags & Drive::ControlFlag::BRICK;
    
    driveCtrlFlags = Drive::ControlFlag::NONE;

    if (!driveRobotCentric) {
        driveCtrlFlags |= Drive::ControlFlag::FIELD_CENTRIC;
    }

    if (brickDrive) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    if (viceGrip) {
        driveCtrlFlags |= Drive::ControlFlag::VICE_GRIP;
    }

    if (driveRecording) {
        driveCtrlFlags |= Drive::ControlFlag::RECORDING;
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

    double finalXVel = 0.0,
           finalYVel = 0.0,
           finalAngVel = 0.0;

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

    // Returns whether the robot should be moving.
    auto isMoving = [&]() -> bool {
        return (driveCtrlFlags & Drive::ControlFlag::VICE_GRIP) || finalXVel || finalYVel || finalAngVel;
    };

    // Stay in brick drive mode if the robot isn't moving.
    if (wasBrickDrive && !isMoving()) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    // Control the drivetrain.
    drive->manualControl(finalXVel, -finalYVel, -finalAngVel, driveCtrlFlags);
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
    driveRobotCentric = switchPanel.GetRawButton(2);
    driveRecording = switchPanel.GetRawButton(3);

    if (blinkyBlinky) {
        if (switchPanel.GetRawButton(4)) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::OFF);
        }
        else if (!drive->isIMUCalibrated()) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CALIBRATING);
        }
        else if (getCurrentMode() == MatchMode::DISABLED) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::DISABLED);
        }
        else if (driveRecording) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::RECORDING);
        }
        else if (settings.isCraterMode) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CRATER_MODE);
        }
        else if (driveCtrlFlags & Drive::ControlFlag::BRICK) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::BRICK);
        }
        else if (driveCtrlFlags & Drive::ControlFlag::VICE_GRIP) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::VICE_GRIP);
        }
        else {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::ALLIANCE);
        }
    }
}

void Controls::sendFeedback() { }
