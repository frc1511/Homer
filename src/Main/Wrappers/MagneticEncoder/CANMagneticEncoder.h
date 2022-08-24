#pragma once

#include <units/angle.h>
#include <units/time.h>

class ThunderCANMagneticEncoder {
public:
    ThunderCANMagneticEncoder(int canID) { }
    virtual ~ThunderCANMagneticEncoder() = default;

    virtual int configFactoryDefault(units::millisecond_t timeout = 50_ms) { return 0; }

    enum class SensorRange {
        UNSIGNED_0_TO_360,
        SIGNED_PLUS_MINUS_180,
    };

    virtual int configAbsoluteSensorRange(SensorRange range, units::millisecond_t timeout = 50_ms) { return 0; }

    virtual units::degree_t getAbsolutePosition() { return 0_deg; }
};