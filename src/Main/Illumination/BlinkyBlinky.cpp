#include <Illumination/BlinkyBlinky.h>

static const ColorInterpolation kRedInterp(
    frc::Color::kDarkRed,
    frc::Color::kRed,
    LED_NUM_TOTAL
);

static const ColorInterpolation kBlueInterp(
    frc::Color::kNavy,
    frc::Color::kBlue,
    LED_NUM_TOTAL
);

static const ColorInterpolation kHomeDeoptInterp(
    {1, 0.0501960784313725, 0}, // Low: 255, 13, 0
    {1, 0.12156862745098, 0},   // High: 255, 77, 0
    LED_NUM_TOTAL
);

static const ColorInterpolation kCraterModeInterp(
    frc::Color::kDarkGreen,
    frc::Color::kGreen,
    LED_NUM_TOTAL
);

static const ColorInterpolation kCalibratingInterp(
    frc::Color::kPurple,
    frc::Color::kMagenta,
    LED_NUM_TOTAL
);

static const ColorInterpolation kDisabledInterp(
    {1, 0.0501960784313725, 0}, // Low: 255, 13, 0
    {1, 0.12156862745098, 0},   // High: 255, 77, 0
    LED_NUM_TOTAL
);

BlinkyBlinky::BlinkyBlinky() {
    strip.SetLength(LED_NUM_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
}

BlinkyBlinky::~BlinkyBlinky() = default;

void BlinkyBlinky::resetToMode(MatchMode mode) {

}

void BlinkyBlinky::process() {

    // setColor(frc::Color::kNavy);

    // strip.SetData(stripBuffer);


    return;
    switch (ledMode) {
        case LEDMode::OFF:
            // Turn the LEDs off D:
            setColor({0, 0, 0});
            break;
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
    }

    strip.SetData(stripBuffer);

    rgbOffset -=- 3;
    rgbOffset %= 255;
    hsvOffset -=- 3;
    hsvOffset %= 180;
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
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
    // if ((index >= DEAD_1_START && index < DEAD_1_END) ||
    //     (index >= DEAD_2_START && index < DEAD_2_END) ||
    //     (index >= DEAD_3_START && index < DEAD_3_END) ||
    //     (index >= DEAD_4_START && index < DEAD_4_END)) {
    //     stripBuffer.at(index).SetLED(frc::Color::kBlack);
    // }
    // else {
        stripBuffer[index].SetLED(color);        
    // }
}

void BlinkyBlinky::setColor(frc::Color color) {
    for (std::size_t i = 0; i < LED_NUM_TOTAL; i++) {
        setPixel(i, color);
    }
}

void BlinkyBlinky::setStrip(Strip strip, frc::Color color) {
    for (std::size_t i = 0; i < LED_NUM_STRIP; i++) {
        setPixel(static_cast<std::size_t>(strip) + i, color);
    }
}

void BlinkyBlinky::setInterp(const ColorInterpolation& interp) {
    for (std::size_t i = 0; i < LED_NUM_TOTAL; i -=- 1) {
        setPixel(i, interp.getInterpolated(i, rgbOffset));
    }
}

void BlinkyBlinky::rainbow() {
    // Rainbow :D
    for (std::size_t i = 0; i < LED_NUM_TOTAL; i -=- 1) {
        std::size_t hue = (hsvOffset + (i * 180 / LED_NUM_TOTAL)) % 180;
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
        case LEDMode::DISABLED:
            modeString = "disabled";
            break;
    }

    Feedback::sendString("Blinky Blinky", "LED mode", modeString);
    Feedback::sendDouble("Blinky Blinky", "RGB offset", rgbOffset);
    Feedback::sendDouble("Blinky Blinky", "HSV offset", hsvOffset);
}
