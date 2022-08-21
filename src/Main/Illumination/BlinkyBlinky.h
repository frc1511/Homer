#pragma once

#include <Basic/Mechanism.h>
#include <Hardware/IOMap.h>
#include <Basic/Feedback.h>
#include <Illumination/ColorInterpolation.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>
#include <map>

#define LED_NUM_STRIP 20
#define NUM_STRIPS 4
#define LED_NUM_TOTAL 20//(NUM_STRIPS * LED_NUM_STRIP)
// #define LED_NUM_TOTAL 40

class BlinkyBlinky : public Mechanism {
public:
    BlinkyBlinky();
    ~BlinkyBlinky();

    void resetToMode(MatchMode mode) override;
    void process() override;
    void sendFeedback() override;

    enum class LEDMode {
        OFF,
        RAINBOW,
        ALLIANCE,
        HOME_DEPOT,
        CRATER_MODE,
        CALIBRATING,
        BRICK,
        VICE_GRIP,
        RECORDING,
        DISABLED,
    };

    void setLEDMode(LEDMode mode);

private:
    frc::AddressableLED strip { PWM_BLINKY_BLINKY };

    std::array<frc::AddressableLED::LEDData, LED_NUM_TOTAL> stripBuffer;

    enum class Strip : std::size_t {
        FRONT = 0,
        RIGHT = 40,
        BACK = 80,
        LEFT = 120,
    };

    void setPixel(std::size_t index, frc::Color color);
    void setColor(frc::Color color);
    void setStrip(Strip strip, frc::Color color);
    void setInterp(const ColorInterpolation& interp);
    void rainbow();

    LEDMode ledMode = LEDMode::ALLIANCE;

    std::size_t rgbOffset = 0,
                hsvOffset = 0;
};
