#pragma once

#include <Util/Interpolation.h>
#include <frc/util/Color.h>

class ColorInterpolation {
public:
    ColorInterpolation(frc::Color low, frc::Color high, std::size_t ledNum);
    ~ColorInterpolation();

    frc::Color getInterpolated(std::size_t index, std::size_t offset) const;

private:
    Interpolation<double, double> r, g, b;

    std::size_t ledNum;
};