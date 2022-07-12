#include <Illumination/ColorInterpolation.h>

ColorInterpolation::ColorInterpolation(frc::Color low, frc::Color high, std::size_t _ledNum)
: r({ { 0, low.red   }, { 254, high.red   } }),
  g({ { 0, low.green }, { 254, high.green } }),
  b({ { 0, low.blue  }, { 254, high.blue  } }),
  ledNum(_ledNum) { }

ColorInterpolation::~ColorInterpolation() = default;

frc::Color ColorInterpolation::getInterpolated(std::size_t index, std::size_t offset) const {
    std::size_t x = (offset + (index * 255 / ledNum)) % 255;
    return { r[x].value(), g[x].value(), b[x].value() };
}
