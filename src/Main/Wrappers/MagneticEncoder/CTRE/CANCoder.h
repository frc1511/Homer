#pragma once

#include <Wrappers/MagneticEncoder/CANMagneticEncoder.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class ThunderCANCoder : public ThunderCANMagneticEncoder {
public:
    ThunderCANCoder(int canID);
    ~ThunderCANCoder();

    int configFactoryDefault(units::millisecond_t timeout = 50_ms);

    int configAbsoluteSensorRange(SensorRange range, units::millisecond_t timeout = 50_ms);

    units::degree_t getAbsolutePosition();

private:
    ctre::phoenix::sensors::CANCoder canCoder;
};