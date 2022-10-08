#include <Wrappers/MotorController/Rev/CANSparkMax.h>
#include <cassert>

ThunderCANSparkMax::ThunderCANSparkMax(int canID)
: ThunderCANMotorController(canID),
  sparkMax(canID, rev::CANSparkMax::MotorType::kBrushless),
  encoder(sparkMax.GetEncoder()),
  alternateEncoder(std::nullopt),
  pidController(sparkMax.GetPIDController()) { }

ThunderCANSparkMax::~ThunderCANSparkMax() = default;

ThunderCANMotorController::Vendor ThunderCANSparkMax::getVendor() const {
    return ThunderCANMotorController::Vendor::REV;
}

int ThunderCANSparkMax::set(ControlMode mode, double value) {
    rev::REVLibError error = rev::REVLibError::kOk;
    
    switch (mode) {
        case ControlMode::PERCENT_OUTPUT:
            sparkMax.Set(value);
            break;
        case ControlMode::POSITION:
            error = pidController.SetReference(value, rev::CANSparkMax::ControlType::kPosition);
            break;
        case ControlMode::VELOCITY:
            error = pidController.SetReference(value, rev::CANSparkMax::ControlType::kVelocity);
            break;
        case ControlMode::CURRENT:
            error = pidController.SetReference(value, rev::CANSparkMax::ControlType::kCurrent);
            break;
    }
    
    return static_cast<int>(error);
}

double ThunderCANSparkMax::getPercentOutput() const {
    return sparkMax.Get();
}

double ThunderCANSparkMax::getEncoderPosition() const {
    return encoder.GetPosition();
}

double ThunderCANSparkMax::getEncoderVelocity() const {
    return encoder.GetVelocity();
}

int ThunderCANSparkMax::setEncoderPosition(double position) {
    rev::REVLibError error = encoder.SetPosition(position);

    return static_cast<int>(error);
}

double ThunderCANSparkMax::getAlternateEncoderPosition() {
    assert(alternateEncoder.has_value());
    
    return alternateEncoder.value().GetPosition();
}

double ThunderCANSparkMax::getAlternateEncoderVelocity() {
    assert(alternateEncoder.has_value());
    
    return alternateEncoder.value().GetVelocity();
}

int ThunderCANSparkMax::setAlternateEncoderPosition(double position) {
    assert(alternateEncoder.has_value());
    
    rev::REVLibError error = alternateEncoder.value().SetPosition(position);
    
    return static_cast<int>(error);
}

int ThunderCANSparkMax::setIdleMode(IdleMode mode) {
    rev::CANSparkMax::IdleMode revIdleMode = rev::CANSparkMax::IdleMode::kBrake;
    switch (mode) {
        case IdleMode::BRAKE:
            revIdleMode = rev::CANSparkMax::IdleMode::kBrake;
            break;
        case IdleMode::COAST:
            revIdleMode = rev::CANSparkMax::IdleMode::kCoast;
            break;
    }
    
    rev::REVLibError error = sparkMax.SetIdleMode(revIdleMode);
    
    return static_cast<int>(error);
}

void ThunderCANSparkMax::setInverted(bool isInverted) {
    sparkMax.SetInverted(isInverted);
}

bool ThunderCANSparkMax::getInverted() const {
    return sparkMax.GetInverted();
}

units::ampere_t ThunderCANSparkMax::getOutputCurrent() const {
    return units::ampere_t(sparkMax.GetOutputCurrent());
}

int ThunderCANSparkMax::configFactoryDefault(units::millisecond_t timeout) {
    rev::REVLibError error = sparkMax.RestoreFactoryDefaults();
    
    return static_cast<int>(error);
}

int ThunderCANSparkMax::configOpenLoopRamp(units::second_t seconds, units::millisecond_t timeout) {
    rev::REVLibError error = sparkMax.SetOpenLoopRampRate(seconds.value());
    
    return static_cast<int>(error);
}

int ThunderCANSparkMax::configClosedLoopRamp(units::second_t seconds, units::millisecond_t timeout) {
    rev::REVLibError error = sparkMax.SetClosedLoopRampRate(seconds.value());
    
    return static_cast<int>(error);
}

int ThunderCANSparkMax::configSmartCurrentLimit(units::ampere_t limit, units::millisecond_t timeout) {
    rev::REVLibError error = sparkMax.SetSmartCurrentLimit(limit.value());
    
    return static_cast<int>(error);
}

int ThunderCANSparkMax::burnFlash() {
    rev::REVLibError error = sparkMax.BurnFlash();
    
    return static_cast<int>(error);
}

int ThunderCANSparkMax::configAlternateEncoder(int countsPerRev) {
    alternateEncoder = sparkMax.GetAlternateEncoder(countsPerRev);
    
    return 0;
}

units::fahrenheit_t ThunderCANSparkMax::getTemperature() const {
    return units::celsius_t(sparkMax.GetMotorTemperature());
}

int ThunderCANSparkMax::configP(double gain, int slotID, units::millisecond_t timeout) {
    rev::REVLibError error = pidController.SetP(gain, slotID);

    return static_cast<int>(error);
}

int ThunderCANSparkMax::configI(double gain, int slotID, units::millisecond_t timeout) {
    rev::REVLibError error = pidController.SetI(gain, slotID);

    return static_cast<int>(error);
}

int ThunderCANSparkMax::configD(double gain, int slotID, units::millisecond_t timeout) {
    rev::REVLibError error = pidController.SetD(gain, slotID);

    return static_cast<int>(error);
}

int ThunderCANSparkMax::configFF(double gain, int slotID, units::millisecond_t timeout) {
    rev::REVLibError error = pidController.SetFF(gain, slotID);

    return static_cast<int>(error);
}

int ThunderCANSparkMax::configIZone(double gain, int slotID, units::millisecond_t timeout) {
    rev::REVLibError error = pidController.SetIZone(gain, slotID);

    return static_cast<int>(error);
}

int ThunderCANSparkMax::configOutputRange(double min, double max, int slotID, units::millisecond_t timeout) {
    rev::REVLibError error = pidController.SetOutputRange(min, max, slotID);
    
    return static_cast<int>(error);
}

void ThunderCANSparkMax::follow(ThunderCANMotorController* master) {
    assert(master && master->getVendor() == ThunderCANMotorController::Vendor::REV);
    
    auto sparkMaxMaster = reinterpret_cast<rev::CANSparkMax*>(master->getRawMotorController());
    sparkMax.Follow(*sparkMaxMaster);
}

void* ThunderCANSparkMax::getRawMotorController() {
    return reinterpret_cast<void*>(&sparkMax);
}
