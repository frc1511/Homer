#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

class ThunderIMU {
public:
    ThunderIMU() = default;
    virtual ~ThunderIMU() = default;

    virtual void calibrate() { }

    enum class CalibrationTime {
        _32ms = 0, _64ms = 1, _128ms = 2, _256ms = 3,
        _512ms = 4, _1s = 5, _2s = 6, _4s = 7,
        _8s = 8, _16s = 9, _32s = 10, _64s = 11
    };

    virtual int configCalTime(CalibrationTime time)              { return 0; }
    virtual void reset()                                         { }
    virtual units::degree_t getAngle() const                     { return 0_deg; }
    virtual units::degrees_per_second_t getRate() const          { return 0_deg_per_s; }
    virtual units::meters_per_second_squared_t getAccelX() const { return 0_mps_sq; }
    virtual units::meters_per_second_squared_t getAccelY() const { return 0_mps_sq; }
    virtual units::meters_per_second_squared_t getAccelZ() const { return 0_mps_sq; }

    enum class Axis { X, Y, Z };

    virtual Axis getYawAxis() const      { return Axis::Z; }
    virtual int setYawAxis(Axis yawAxis) { return 0; }
    virtual int getPort() const          { return -1; }
};