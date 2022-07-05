#pragma once

#include <Wrappers/MotorController/MotorController.h>

class ThunderMotorControllerEmpty: public ThunderMotorController {
public:
    inline ThunderMotorControllerEmpty(int canID) { }
    inline ~ThunderMotorControllerEmpty() = default;

    inline Vendor getVendor() const                                                                            { return Vendor::UNKNOWN; }
    inline int set(ControlMode mode, double value)                                                             { return 0; }
    inline double getPercentOutput() const                                                                     { return 0.0; }
    inline double getEncoderPosition()                                                                         { return 0.0; }
    inline double getEncoderVelocity() const                                                                   { return 0.0; }
    inline int setEncoderPosition(double position)                                                             { return 0; }
    inline double getAlternateEncoderPosition()                                                                { return 0.0; }
    inline double getAlternateEncoderVelocity()                                                                { return 0.0; }
    inline int setAlternateEncoderPosition(double position)                                                    { return 0; }
    inline int setIdleMode(IdleMode mode)                                                                      { return 0; }
    inline void setInverted(bool isInverted)                                                                   { }
    inline bool getInverted() const                                                                            { return false; }
    inline int configFactoryDefault(units::millisecond_t timeout = 50_ms)                                      { return 0; }
    inline int configOpenLoopRamp(units::second_t seconds, units::millisecond_t timeout = 50_ms)               { return 0; }
    inline int configClosedLoopRamp(units::second_t seconds, units::millisecond_t timeout = 50_ms)             { return 0; }
    inline int configSmartCurrentLimit(units::ampere_t limit, units::millisecond_t timeout = 50_ms)            { return 0; }
    inline int burnFlash()                                                                                     { return 0; }
    inline int configAlternateEncoder(int countsPerRev)                                                        { return 0; }
    inline units::fahrenheit_t getTemperature() const                                                          { return units::fahrenheit_t(32); }
    inline int configP(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms)                      { return 0; }
    inline int configI(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms)                      { return 0; }
    inline int configD(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms)                      { return 0; }
    inline int configFF(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms)                     { return 0; }
    inline int configIZone(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms)                  { return 0; }
    inline int configOutputRange(double min, double max, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }
    inline void follow(ThunderMotorController* master)                                                         { }
    inline void* getRawMotorController()                                                                       { return nullptr; }
};
