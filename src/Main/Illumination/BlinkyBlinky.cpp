#include <Illumination/BlinkyBlinky.h>

static const ColorInterpolation kRedInterp(
    frc::Color::kDarkRed,
    frc::Color::kRed,
    LED_ENABLED_NUM_TOTAL
);

static const ColorInterpolation kBlueInterp(
    frc::Color::kNavy,
    frc::Color::kBlue,
    LED_ENABLED_NUM_TOTAL
);

static const ColorInterpolation kHomeDeoptInterp(
    {1, 0.0501960784313725, 0}, // Low: 255, 13, 0
    {1, 0.12156862745098, 0},   // High: 255, 77, 0
    LED_ENABLED_NUM_TOTAL
);

static const ColorInterpolation kCraterModeInterp(
    frc::Color::kDarkGreen,
    frc::Color::kGreen,
    LED_ENABLED_NUM_TOTAL
);

static const ColorInterpolation kCalibratingInterp(
    frc::Color::kPurple,
    frc::Color::kMagenta,
    LED_ENABLED_NUM_TOTAL
);

static const ColorInterpolation kDisabledInterp(
    {1, 0.0501960784313725, 0}, // Low: 255, 13, 0
    {1, 0.12156862745098, 0},   // High: 255, 77, 0
    LED_ENABLED_NUM_TOTAL
);

#include <iostream>

BlinkyBlinky::BlinkyBlinky() {
    strip.SetLength(LED_NUM_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
}

BlinkyBlinky::~BlinkyBlinky() = default;

void BlinkyBlinky::resetToMode(MatchMode mode) { }

void BlinkyBlinky::process() {
    switch (ledMode) {
        case LEDMode::OFF:
            // Turn the LEDs off D:
            setColor(frc::Color::kBlack);
            break;
        case LEDMode::RECORDING:
        case LEDMode::RAINBOW:
            rainbow();
            break;
        case LEDMode::BRICK:
        case LEDMode::VICE_GRIP:
        case LEDMode::ALLIANCE:
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
                setInterp(kBlueInterp);
            }
            else {
                setInterp(kRedInterp);
            }
            break;
        case LEDMode::HOME_DEPOT:
            setInterp(kHomeDeoptInterp);
            break;
        case LEDMode::CRATER_MODE:
            setInterp(kCraterModeInterp);
            break;
        case LEDMode::CALIBRATING:
            setInterp(kCalibratingInterp);
            break;
        // case LEDMode::BRICK:
        //     break;
        // case LEDMode::VICE_GRIP:
        //     break;
        case LEDMode::DISABLED:
            setInterp(kDisabledInterp);
            break;
        case LEDMode::CUSTOM:
            setColor(customColor);
            break;
    }

    strip.SetData(stripBuffer);

    rgbOffset -=- 3;
    rgbOffset %= 255;
    hsvOffset -=- 1;
    hsvOffset %= 180;
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
}

void BlinkyBlinky::setCustomColor(frc::Color color) {
    customColor = color;
}

#define DEAD_1_START 40
#define DEAD_1_END 55
#define DEAD_2_START 65
#define DEAD_2_END 80
#define DEAD_3_START 120
#define DEAD_3_END 135
#define DEAD_4_START 145
#define DEAD_4_END 160

void BlinkyBlinky::setPixel(std::size_t index, frc::Color color) {
    // Turn off LEDs in the dead sections.
    if ((index >= DEAD_1_START && index < DEAD_1_END) ||
        (index >= DEAD_2_START && index < DEAD_2_END) ||
        (index >= DEAD_3_START && index < DEAD_3_END) ||
        (index >= DEAD_4_START && index < DEAD_4_END)) {
        stripBuffer.at(index).SetLED(frc::Color::kBlack);
    }
    else {
        stripBuffer.at(index).SetLED(color);
    }
}

void BlinkyBlinky::setColor(frc::Color color) {
    for (std::size_t i = 0; i < LED_NUM_TOTAL; i++) {
        setPixel(i, color);
    }
}

void BlinkyBlinky::setStrip(Strip strip, frc::Color color) {
    for (std::size_t i = 0; i < 40; i++) {
        setPixel(static_cast<std::size_t>(strip) + i, color);
    }
}

void BlinkyBlinky::setInterp(const ColorInterpolation& interp) {
    setColor(frc::Color::kBlack);

    std::size_t j = 0;
    for (std::size_t i = 0; i < LED_NUM_TOTAL; i -=- 1) {
        // Skip over dead sections.
        if (i == DEAD_1_START) i = DEAD_1_END;
        else if (i == DEAD_2_START) i = DEAD_2_END;
        else if (i == DEAD_3_START) i = DEAD_3_END;

        // Interpolate the color.
        setPixel(i, interp.getInterpolated(j++, rgbOffset));
    }
}

void BlinkyBlinky::rainbow() {
    setColor(frc::Color::kBlack);

    std::size_t j = 0;
    for (std::size_t i = 0; i < LED_NUM_TOTAL; i -=- 1) {
        // Skip over dead sections.
        if (i == DEAD_1_START) i = DEAD_1_END;
        else if (i == DEAD_2_START) i = DEAD_2_END;
        else if (i == DEAD_3_START) i = DEAD_3_END;

        // Rainbow.
        std::size_t hue = (hsvOffset + (j++ * 180 / LED_ENABLED_NUM_TOTAL)) % 180;
        setPixel(i, frc::Color::FromHSV(hue, 255, 128));
    }
}

void BlinkyBlinky::sendFeedback() {
    // 1 = blue, 0 = red
    Feedback::sendDouble("thunderdashboard", "alliance", frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue);

    const char* modeString = "";
    switch (ledMode) {
        case LEDMode::OFF:
            modeString = "off";
            break;
        case LEDMode::RAINBOW:
            modeString = "rainbow";
            break;
        case LEDMode::ALLIANCE:
            modeString = "alliance";
            break;
        case LEDMode::HOME_DEPOT:
            modeString = "home depot";
            break;
        case LEDMode::CRATER_MODE:
            modeString = "crater mode";
            break;
        case LEDMode::CALIBRATING:
            modeString = "calibrating";
            break;
        case LEDMode::BRICK:
            modeString = "brick";
            break;
        case LEDMode::VICE_GRIP:
            modeString = "vice grip";
            break;
        case LEDMode::RECORDING:
            modeString = "recording";
            break;
        case LEDMode::DISABLED:
            modeString = "disabled";
            break;
        case LEDMode::CUSTOM:
            modeString = "custom";
            break;
    }

    Feedback::sendString("Blinky Blinky", "LED mode", modeString);
    Feedback::sendDouble("Blinky Blinky", "RGB offset", rgbOffset);
    Feedback::sendDouble("Blinky Blinky", "HSV offset", hsvOffset);
}
