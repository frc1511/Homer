#include <Wrappers/MagneticEncoder/CTRE/CANCoder.h>

ThunderCANCoder::ThunderCANCoder(int canID)
: ThunderCANMagneticEncoder(canID), canCoder(canID) { }

ThunderCANCoder::~ThunderCANCoder() = default;

int ThunderCANCoder::configFactoryDefault(units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = canCoder.ConfigFactoryDefault(timeout.value());

    return static_cast<int>(error);
}

int ThunderCANCoder::configAbsoluteSensorRange(SensorRange range, units::millisecond_t timeout) {
    ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    
    if (range == ThunderCANMagneticEncoder::SensorRange::SIGNED_PLUS_MINUS_180) {
        absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
    }
    
    ctre::phoenix::ErrorCode error = canCoder.ConfigAbsoluteSensorRange(absoluteSensorRange, timeout.value());

    return static_cast<int>(error);
}

units::degree_t ThunderCANCoder::getAbsolutePosition() {
    return units::degree_t(canCoder.GetAbsolutePosition());
}