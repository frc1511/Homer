#include <Vision/Limelight.h>

#define LIMELIGHT_ANGLE 29.3_deg // the limelight is really 22.4 degrees Farenheight

#define LIMELIGHT_HEIGHT 43_in

Limelight::Limelight()
: table(nt::NetworkTableInstance::GetDefault().GetTable(LIMELIGHT_NT_NAME)) { }

Limelight::~Limelight() = default;

void Limelight::resetToMode(MatchMode mode) {
    switch (mode) {
        case MatchMode::DISABLED:
        case MatchMode::TEST:
            setLEDMode(LEDMode::OFF);
            break;
        case MatchMode::AUTO:
        case MatchMode::TELEOP:
            setLEDMode(LEDMode::ON);
            break;
        }
}

void Limelight::sendFeedback() {
    Feedback::sendDouble("Limelight", "horizontal angle (radians)", getAngleHorizontal().value());
    Feedback::sendDouble("Limelight", "vertical angle (radians)", getAngleVertical().value());
}

bool Limelight::hasTarget() {
    return table->GetNumber("tv", 0.0);
}

units::radian_t Limelight::getAngleHorizontal() {
    return units::degree_t(table->GetNumber("tx", 0.0));
}

units::radian_t Limelight::getAngleVertical() {
    return units::degree_t(table->GetNumber("ty", 0.0));
}

units::meter_t Limelight::getDistance(units::meter_t targetHeight) {
    if (!hasTarget()) return -1_m;

    return units::meter_t(units::meter_t(targetHeight - LIMELIGHT_HEIGHT).value() / units::math::tan(LIMELIGHT_ANGLE + getAngleVertical()).value());
}

void Limelight::setLEDMode(LEDMode mode) {
    table->PutNumber("ledMode", static_cast<int>(mode));
}

Limelight::LEDMode Limelight::getLEDMode() {
    return static_cast<LEDMode>(table->GetNumber("ledMode", 0.0));
}

void Limelight::setCameraMode(CameraMode mode) {
    table->PutNumber("camMode", static_cast<int>(mode));
}

Limelight::CameraMode Limelight::getCameraMode() {
    return static_cast<CameraMode>(table->GetNumber("camMode", 0.0));
}