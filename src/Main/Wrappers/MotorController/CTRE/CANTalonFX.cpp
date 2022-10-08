#include <Wrappers/MotorController/CTRE/CANTalonFX.h>

ThunderCANTalonFX::ThunderCANTalonFX(int canID)
: ThunderCANMotorController(canID), talon(canID) { }

ThunderCANTalonFX::~ThunderCANTalonFX() = default;

ThunderCANMotorController::Vendor ThunderCANTalonFX::getVendor() const {
    return ThunderCANMotorController::Vendor::CTRE;
}

int ThunderCANTalonFX::set(ControlMode mode, double value) {
    TalonFXControlMode talonCtrlMode = TalonFXControlMode::PercentOutput;
    switch (mode) {
        case ControlMode::PERCENT_OUTPUT:
            talonCtrlMode = TalonFXControlMode::PercentOutput;
            break;
        case ControlMode::POSITION:
            talonCtrlMode = TalonFXControlMode::Position;
            break;
        case ControlMode::VELOCITY:
            talonCtrlMode = TalonFXControlMode::Velocity;
            break;
        case ControlMode::CURRENT:
            talonCtrlMode = TalonFXControlMode::Current;
            break;
    }
    
    talon.Set(talonCtrlMode, value);
    return 0;
}

double ThunderCANTalonFX::getPercentOutput() const {
    return talon.GetMotorOutputPercent();
}

double ThunderCANTalonFX::getEncoderPosition() const {
    return talon.GetSelectedSensorPosition(0);
}

double ThunderCANTalonFX::getEncoderVelocity() const {
    return talon.GetSelectedSensorVelocity(0);
}

int ThunderCANTalonFX::setEncoderPosition(double position) {
    talon.SetSelectedSensorPosition(position, 0);
    return 0;
}

double ThunderCANTalonFX::getAlternateEncoderPosition() {
    return talon.GetSelectedSensorPosition(1);
}

double ThunderCANTalonFX::getAlternateEncoderVelocity() {
    return talon.GetSelectedSensorVelocity(1);
}

int ThunderCANTalonFX::setAlternateEncoderPosition(double position) {
    talon.SetSelectedSensorPosition(position, 1);
    return 0;
}

int ThunderCANTalonFX::setIdleMode(IdleMode mode) {
    NeutralMode talonNeutralMode = NeutralMode::Brake;
    switch (mode) {
        case IdleMode::BRAKE:
            talonNeutralMode = NeutralMode::Brake;
            break;
        case IdleMode::COAST:
            talonNeutralMode = NeutralMode::Coast;
            break;
    }
    
    talon.SetNeutralMode(talonNeutralMode);
    return 0;
}

void ThunderCANTalonFX::setInverted(bool isInverted) {
    talon.SetInverted(isInverted);
}

bool ThunderCANTalonFX::getInverted() const {
    return talon.GetInverted();
}

units::ampere_t ThunderCANTalonFX::getOutputCurrent() const {
    return units::ampere_t(talon.GetOutputCurrent());
}

int ThunderCANTalonFX::configFactoryDefault(units::millisecond_t timeout) {
   ctre::phoenix::ErrorCode error = talon.ConfigFactoryDefault(timeout.value());

   return static_cast<int>(error);
}

int ThunderCANTalonFX::configOpenLoopRamp(units::second_t seconds, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.ConfigOpenloopRamp(seconds.value(), timeout.value());
    
   return static_cast<int>(error);
}

int ThunderCANTalonFX::configClosedLoopRamp(units::second_t seconds, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.ConfigClosedloopRamp(seconds.value(), timeout.value());
    
    return static_cast<int>(error);
}

int ThunderCANTalonFX::configSmartCurrentLimit(units::ampere_t limit, units::millisecond_t timeout) {
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;
    talon.GetAllConfigs(config);

    config.supplyCurrLimit.currentLimit = limit.value();
    ctre::phoenix::ErrorCode error = talon.ConfigSupplyCurrentLimit(config.supplyCurrLimit, timeout.value());
    
    return static_cast<int>(error);
}

int ThunderCANTalonFX::burnFlash() {
    return 0;
}

int ThunderCANTalonFX::configAlternateEncoder(int countsPerRev) {
    return 0;
}

units::fahrenheit_t ThunderCANTalonFX::getTemperature() const {
    return units::celsius_t(talon.GetTemperature());
}

int ThunderCANTalonFX::configP(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kP(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonFX::configI(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kI(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonFX::configD(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kD(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonFX::configFF(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_kF(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonFX::configIZone(double gain, int slotID, units::millisecond_t timeout) {
    ctre::phoenix::ErrorCode error = talon.Config_IntegralZone(slotID, gain, timeout.value());

    return static_cast<int>(error);
}

int ThunderCANTalonFX::configOutputRange(double min, double max, int slotID, units::millisecond_t timeout) {
    // I am pretty sure this is right, but can't be tested right now.
    ctre::phoenix::ErrorCode error1 = talon.ConfigPeakOutputForward(max, timeout.value());
    ctre::phoenix::ErrorCode error2 = talon.ConfigPeakOutputReverse(min, timeout.value());
    
    int res = static_cast<int>(error1);
    if (!res) res = static_cast<int>(error2);
    
    return res;
}

void ThunderCANTalonFX::follow(ThunderCANMotorController* master) {
    assert(master && master->getVendor() == ThunderCANMotorController::Vendor::CTRE);
    
    auto talonMaster = reinterpret_cast<ctre::phoenix::motorcontrol::can::TalonFX*>(master->getRawMotorController());
    talon.Follow(*talonMaster);
}

void* ThunderCANTalonFX::getRawMotorController() {
    return reinterpret_cast<void*>(&talon);
}
