#pragma once

#include <Hardware/IOMap.h>
#include <Basic/Settings.h>

class Mechanism {
public:
    virtual ~Mechanism();

    enum class MatchMode {
        DISABLED,
        AUTO,
        TELEOP,
        TEST,
    };
    
    /**
     * The mechanism should do any necessary calibrations and then
     * reset and configure any and all hardware resources for correct
     * "power on" operation, saving such in non-volatile/persistent storage.
     *
     * Any invocation of this will always be followed by an invocation of
     * resetToMode() to restore correct "runtime" configurations for
     * operational match modes.
     */
    virtual void doPersistentConfiguration();

    /**
     * Reset this mechanism in preparation to run in the given mode of a match
     * This should reset all internal state to make the system ready to operate
     * in the given match mode on the next set of commands.  Generally speaking,
     * a mechanism will abandon all commands in progress when this is called
     * and reset to a default state.
     */
    virtual void resetToMode(MatchMode mode);

    /**
     * Send operational and/or diagnostic feedback to the operator of the robot.
     * This is called periodically in all modes of competition.
     */
    virtual void sendFeedback();

    /**
     * Mechanisms should implement this to perform periodic actions in response to
     * commands given to the mechanism. This is called periodically in all modes
     * of competition except for disabled.
     */
    virtual void process();

    static Settings settings;

protected:
    MatchMode getCurrentMode();
};