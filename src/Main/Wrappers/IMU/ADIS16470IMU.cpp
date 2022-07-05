#include <Wrappers/IMU/ADIS16470IMU.h>

ThunderADIS16470IMU::ThunderADIS16470IMU()
: imu() { }

ThunderADIS16470IMU::~ThunderADIS16470IMU() = default;

void ThunderADIS16470IMU::calibrate() {
    imu.Calibrate();
}

int ThunderADIS16470IMU::configCalTime(CalibrationTime time) {
    int result = imu.ConfigCalTime(static_cast<frc::ADIS16470_IMU::CalibrationTime>(time));
    
    return result;
}

void ThunderADIS16470IMU::reset() {
    imu.Reset();
}

units::degree_t ThunderADIS16470IMU::getAngle() const {
    return imu.GetAngle();
}

units::degrees_per_second_t ThunderADIS16470IMU::getRate() const {
    return imu.GetRate();
}

units::meters_per_second_squared_t ThunderADIS16470IMU::getAccelX() const {
    return imu.GetAccelX();
}

units::meters_per_second_squared_t ThunderADIS16470IMU::getAccelY() const {
    return imu.GetAccelY();
}

units::meters_per_second_squared_t ThunderADIS16470IMU::getAccelZ() const {
    return imu.GetAccelZ();
}

ThunderIMU::Axis ThunderADIS16470IMU::getYawAxis() const {
    return static_cast<ThunderIMU::Axis>(imu.GetYawAxis());
}

int ThunderADIS16470IMU::setYawAxis(Axis yawAxis) {
    int result = imu.SetYawAxis(static_cast<frc::ADIS16470_IMU::IMUAxis>(yawAxis));
    
    return result;
}

int ThunderADIS16470IMU::getPort() const {
    return imu.GetPort();
}