#include <Control/Controls.h>
#include <wpi/numbers>
#include <cmath>

#define AXIS_DEADZONE 0.1

Controls::Controls(Drive* drive, Limelight* limelight, BlinkyBlinky* blinkyBlinky)
: drive(drive), limelight(limelight), blinkyBlinky(blinkyBlinky) { }

Controls::~Controls() = default;

void Controls::process() {
    doDrive();
    doAux();
    doSwitchPanel();
}

void Controls::processInDisabled() {
    doSwitchPanel();

    using DriveButton = HardwareManager::DriveGameController::Button;

    bool toggleCamera = driveController.getButtonPressed(DriveButton::SQUARE);
    bool resetOdometry = driveController.getButtonPressed(DriveButton::OPTIONS);
    bool calIMU = driveController.getButtonPressed(DriveButton::SHARE);

    if (toggleCamera) {
        whichCamera = !whichCamera;
    }

    if (resetOdometry) {
        drive->resetOdometry();
    }

    if (calIMU) {
        drive->calibrateIMU();
    }
}

void Controls::doDrive() {
    using DriveButton = HardwareManager::DriveGameController::Button;
    using DriveAxis = HardwareManager::DriveGameController::Axis;

    bool brickDrive = driveController.getButton(DriveButton::CROSS);
    bool viceGrip = driveController.getButton(DriveButton::CIRCLE);
    bool toggleCamera = driveController.getButtonPressed(DriveButton::SQUARE);

    double xVel = driveController.getAxis(DriveAxis::LEFT_X);
    double yVel = driveController.getAxis(DriveAxis::LEFT_Y);
    double angVel = driveController.getAxis(DriveAxis::RIGHT_X);

    bool resetOdometry = driveController.getButtonPressed(DriveButton::OPTIONS);
    bool calIMU = driveController.getButtonPressed(DriveButton::SHARE);

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

    if (toggleCamera) {
        whichCamera = !whichCamera;
    }

    if (resetOdometry) {
        drive->resetOdometry();
    }

    if (calIMU) {
        drive->calibrateIMU();
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

    using DriveButton = HardwareManager::DriveGameController::Button;
    using AuxButton = HardwareManager::AuxGameController::Button;

    return settings.isCraterMode
        && driveController.getButton(DriveButton::TRIANGLE) && driveController.getDPad() == 180
        && auxController.getButton(AuxButton::CROSS) && auxController.getDPad() == 0;
}

void Controls::doAux() {
    // using AuxButton = HardwareManager::AuxGameController::Button;
    // using AuxAxis = HardwareManager::AuxGameController::Axis;

    // D:
}

void Controls::doSwitchPanel() {
    settings.isCraterMode = switchPanel.getButton(1);
    driveRobotCentric = switchPanel.getButton(2);
    driveRecording = switchPanel.getButton(3);

    // Only turn on limelight when broken switch is on or the drivetrain is vice gripping.
    if (switchPanel.getButton(5) || driveCtrlFlags & Drive::ControlFlag::VICE_GRIP) {
        limelight->setLEDMode(Limelight::LEDMode::ON);
        limelight->setCameraMode(Limelight::CameraMode::VISION_PROCESS);
    }
    else {
        limelight->setLEDMode(Limelight::LEDMode::OFF);
        limelight->setCameraMode(Limelight::CameraMode::DRIVER_CAMERA);
    }

    if (blinkyBlinky) {
        if (switchPanel.getButton(4)) {
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
