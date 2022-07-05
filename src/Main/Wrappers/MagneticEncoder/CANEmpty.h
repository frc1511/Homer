#pragma once

#include <Wrappers/MagneticEncoder/CANMagneticEncoder.h>

class ThudnerCANMagneticEncoderEmpty : public ThunderCANMagneticEncoder {
public:
    ThudnerCANMagneticEncoderEmpty(int canID) { }
    ~ThudnerCANMagneticEncoderEmpty() = default;

    inline int configFactoryDefault(units::millisecond_t timeout = 50_ms) { return 0; }

    inline int configAbsoluteSensorRange(SensorRange range, units::millisecond_t timeout = 50_ms) { return 0; }

    inline units::degree_t getAbsolutePosition() { return 0_deg; }
};