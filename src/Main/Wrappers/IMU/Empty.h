#pragma once

#include <Wrappers/IMU/IMU.h>

class ThunderIMUEmpty : public ThunderIMU {
public:
    ThunderIMUEmpty() = default;
    ~ThunderIMUEmpty() = default;

    inline void calibrate()                                     { }
    inline int configCalTime(CalibrationTime time)              { return 0; }
    inline void reset()                                         { }
    inline units::degree_t getAngle() const                     { return 0_deg; }
    inline units::degrees_per_second_t getRate() const          { return 0_deg_per_s; }
    inline units::meters_per_second_squared_t getAccelX() const { return 0_mps_sq; }
    inline units::meters_per_second_squared_t getAccelY() const { return 0_mps_sq; }
    inline units::meters_per_second_squared_t getAccelZ() const { return 0_mps_sq; }
    inline Axis getYawAxis() const                              { return Axis::X; }
    inline int setYawAxis(Axis yaw_axis)                        { return 0; }
    inline int getPort() const                                  { return -1; }
};