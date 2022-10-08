#pragma once

#include <units/time.h>
#include <units/temperature.h>
#include <units/voltage.h>
#include <units/current.h>

class ThunderCANMotorController {
public:
    ThunderCANMotorController(int canID) { }
    virtual ~ThunderCANMotorController() = default;

    enum class Vendor {
        UNKNOWN,
        REV,
        CTRE,
    };

    virtual Vendor getVendor() const { return Vendor::UNKNOWN; }

    enum class ControlMode {
        PERCENT_OUTPUT,
        POSITION,
        VELOCITY,
        CURRENT,
    };

    /**
     * Sets the output of the motor controller.
     */
    virtual int set(ControlMode mode, double value) { return 0; }

    /**
     * Returns the percent output of the motor controller.
     */
    virtual double getPercentOutput() const { return 0.0; }

    /**
     * Returns the position of the encoder (Rotations).
     */
    virtual double getEncoderPosition() const { return 0.0; }

    /**
     * Returns the velocity of the encoder (RPM).
     */
    virtual double getEncoderVelocity() const { return 0.0; }

    /**
     * Set the position of the encoder.
     */
    virtual int setEncoderPosition(double position) { return 0; }

    /**
     * Returns the position of the alternate encoder (Rotations).
     */
    virtual double getAlternateEncoderPosition() { return 0.0; }

    /**
     * Returns the velocity of the alternate encoder (RPM).
     */
    virtual double getAlternateEncoderVelocity() { return 0.0; }

    /**
     * Sets the position of the alternate encoder.
     */
    virtual int setAlternateEncoderPosition(double position) { return 0; }

    enum class IdleMode { BRAKE = 0, COAST = 1 };
    
    /**
     * Set the idle mode of the motor controller.
     */
    virtual int setIdleMode(IdleMode mode) { return 0; }

    /**
     * Sets whether the output of the motor controller is inverted.
     */
    virtual void setInverted(bool isInverted) { }

    /**
     * Returns whether the output of the motor controller is inverted.
     */
    virtual bool getInverted() const { return false; }

    /**
     * Returns the output current of the motor.
     */
    virtual units::ampere_t getOutputCurrent() const { return 0_A; }

    /**
     * Reverts all configurations to factory default values.
     */
    virtual int configFactoryDefault(units::millisecond_t timeout = 50_ms) { return 0; }
    
    /**
     * Configures the open loop ramp rate of the motor controller from neutral to full throttle.
     */
    virtual int configOpenLoopRamp(units::second_t seconds, units::millisecond_t timeout = 50_ms) { return 0; }
    
    /**
     * Configures the open loop ramp rate of the motor controller from neutral to full throttle.
     */
    virtual int configClosedLoopRamp(units::second_t seconds, units::millisecond_t timeout = 50_ms) { return 0; }
    
    /**
     * Configures the current limit in Amps.
     */
    virtual int configSmartCurrentLimit(units::ampere_t limit, units::millisecond_t timeout = 50_ms) { return 0; }

    /**
     * Writes all settings to flash.
     */
    virtual int burnFlash() { return 0; }

    /**
     * Enables use of the alternate encoder with it configured to read a
     * quadrature encoder with the given counts per revolution.
     * This enables use of Set/GetAlternateEncoder(). This also disables 
     * sparkmax limit inputs.
     */
    virtual int configAlternateEncoder(int countsPerRev) { return 0; }

    /**
     * Returns the temperature of the motor.
     */
    virtual units::fahrenheit_t getTemperature() const { return 0_degC; }

    /**
     * Configures the Proportional value of the PID controller.
     */
    virtual int configP(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }

    /**
     * Configures the Integral value of the PID controller.
     */
    virtual int configI(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }

    /**
     * Configures the Derivative value of the PID controller.
     */
    virtual int configD(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }

    /**
     * Configures the Feed Forward value of the PID controller.
     */
    virtual int configFF(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }

    /**
     * Configures the Integral Zone of the PID controller.
     */
    virtual int configIZone(double gain, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }
    
    /**
     * Configures the output range of the PID controller.
     */
    virtual int configOutputRange(double min, double max, int slotID = 0, units::millisecond_t timeout = 50_ms) { return 0; }

    /**
     * Commands the motor controller to follow a master motor controller.
     */
    virtual void follow(ThunderCANMotorController* master) { }

    virtual void* getRawMotorController() { return nullptr; }

    // TODO: Faults and sticky faults.
};
