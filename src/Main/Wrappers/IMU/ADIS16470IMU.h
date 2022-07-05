#pragma once

#include <Wrappers/IMU/IMU.h>
#include <frc/ADIS16470_IMU.h>

class ThunderADIS16470IMU : public ThunderIMU {
public:
    ThunderADIS16470IMU();
    ~ThunderADIS16470IMU();

    void calibrate();
    int configCalTime(CalibrationTime time);
    void reset();
    units::degree_t getAngle() const;
    units::degrees_per_second_t getRate() const;
    units::meters_per_second_squared_t getAccelX() const;
    units::meters_per_second_squared_t getAccelY() const;
    units::meters_per_second_squared_t getAccelZ() const;
    Axis getYawAxis() const;
    int setYawAxis(Axis yaw_axis);
    int getPort() const;
    
private:
    frc::ADIS16470_IMU imu;
};