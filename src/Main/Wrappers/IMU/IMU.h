#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

class ThunderIMU {
public:
    virtual ~ThunderIMU() = default;

    virtual void calibrate() = 0;

    enum class CalibrationTime {
        _32ms = 0, _64ms = 1, _128ms = 2, _256ms = 3,
        _512ms = 4, _1s = 5, _2s = 6, _4s = 7,
        _8s = 8, _16s = 9, _32s = 10, _64s = 11
    };

    virtual int configCalTime(CalibrationTime time) = 0;

    virtual void reset() = 0;

    virtual units::degree_t getAngle() const = 0;

    virtual units::degrees_per_second_t getRate() const = 0;

    virtual units::meters_per_second_squared_t getAccelX() const = 0;

    virtual units::meters_per_second_squared_t getAccelY() const = 0;

    virtual units::meters_per_second_squared_t getAccelZ() const = 0;

    enum class Axis { X, Y, Z };

    virtual Axis getYawAxis() const = 0;

    virtual int setYawAxis(Axis yawAxis) = 0;

    virtual int getPort() const = 0;
};