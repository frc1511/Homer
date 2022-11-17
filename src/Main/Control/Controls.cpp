#include <Control/Controls.h>
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

void Controls::resetToMode(MatchMode mode) {
    driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();    
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
    bool toggleRotation = driveController.getButtonPressed(DriveButton::TRIANGLE);

    double xVel = driveController.getAxis(DriveAxis::LEFT_X);
    double yVel = driveController.getAxis(DriveAxis::LEFT_Y);
    double angVel = driveController.getAxis(DriveAxis::RIGHT_X);

    double xAng = driveController.getAxis(DriveAxis::RIGHT_X);
    double yAng = driveController.getAxis(DriveAxis::RIGHT_Y);

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

    if (toggleRotation) {
        if (!driveAbsRotation) {
            driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
            driveAbsRotation = true;
        }
        else {
            driveAbsRotation = false;
        }
    }

    if (resetOdometry) {
        drive->resetOdometry();
        driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
    }

    if (calIMU) {
        drive->calibrateIMU();
        driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
    }

    double finalXVel = 0.0,
           finalYVel = 0.0,
           finalAngVel = 0.0,
           finalXAng = 0.0,
           finalYAng = 0.0;

    // Improves the joystick axis to be smoother and easier to control.
    auto improveAxis = [](double axis) -> double {
        return axis;
        //return std::sin(axis * (wpi::numbers::pi / 2.0));
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
    if (std::fabs(xAng) > AXIS_DEADZONE) {
        finalXAng = xAng;
    }
    if (std::fabs(yAng) > AXIS_DEADZONE) {
        finalYAng = yAng;
    }

    // Returns whether the robot should be moving.
    auto isMoving = [&]() -> bool {
        bool r = driveAbsRotation ? finalXAng || finalYAng : finalAngVel;
        return (driveCtrlFlags & Drive::ControlFlag::VICE_GRIP) || finalXVel || finalYVel || r;
    };

    // Stay in brick drive mode if the robot isn't moving.
    if (wasBrickDrive && !isMoving()) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    // Control the drivetrain.    
    if (driveAbsRotation) {
        if (finalYAng || finalXAng) {
            driveAbsAngle = units::radian_t(std::atan2(-finalYAng, finalXAng)) - 90_deg;
        }

        drive->manualControlAbsRotation(finalXVel, -finalYVel, driveAbsAngle, driveCtrlFlags);
    }
    else {
        drive->manualControlRelRotation(finalXVel, -finalYVel, -finalAngVel, driveCtrlFlags);
    }

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
        int mode = Feedback::getDouble("LED", "Mode", 0.0);

        if (switchPanel.getButton(4) || mode == 3) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::OFF);
        }
        else if (mode == 0) {
            if (!drive->isIMUCalibrated()) {
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
        else if (mode == 1) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::ALLIANCE);
        }
        else if (mode == 2) {
            blinkyBlinky->setLEDMode(BlinkyBlinky::LEDMode::CUSTOM);
            double r = Feedback::getDouble("LED", "Custom_Color_R", 0.0),
                   g = Feedback::getDouble("LED", "Custom_Color_G", 0.0),
                   b = Feedback::getDouble("LED", "Custom_Color_B", 0.0);
            
            blinkyBlinky->setCustomColor(frc::Color(r, g, b));
        }
    }
}

void Controls::sendFeedback() { }
