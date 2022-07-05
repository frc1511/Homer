#pragma once

#include <units/angle.h>
#include <units/time.h>

class ThunderCANMagneticEncoder {
public:
    virtual ~ThunderCANMagneticEncoder() = default;

    virtual int configFactoryDefault(units::millisecond_t timeout = 50_ms) = 0;

    enum class SensorRange {
        UNSIGNED_0_TO_360,
        SIGNED_PLUS_MINUS_180,
    };

    virtual int configAbsoluteSensorRange(SensorRange range, units::millisecond_t timeout = 50_ms) = 0;

    virtual units::degree_t getAbsolutePosition() = 0;
};