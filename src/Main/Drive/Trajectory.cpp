#include <Drive/Trajectory.h>
#include <fstream>

Trajectory::Trajectory(const char* path) {
    std::string file_str;
    {
        std::ifstream file(path);
        if (!file) return;
        file_str = std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }

    std::string::const_iterator file_iter = file_str.cbegin();

    auto count = [&]() -> std::size_t {
        std::size_t n = 0;
        while (file_iter != file_str.end() && *file_iter != '\n' && *file_iter != ',' && *file_iter != '}') {
        n++;
        file_iter++;
        }
        return n;
    };

    auto get_num = [&]() -> double {
        std::string::const_iterator start = file_iter;
        std::size_t n = count();
        return std::stod(std::string(start, start + n));
    };

    while (file_iter != file_str.cend()) {
        units::second_t time(get_num()); ++file_iter;
        units::meter_t xPos(get_num()); ++file_iter;
        units::meter_t yPos(get_num()); ++file_iter;
        units::meters_per_second_t velocity(get_num()); ++file_iter;
        // frc::Rotation2d rotation(get_num()); ++file_iter;

        states.emplace(time, State{ xPos, yPos, velocity, frc::Rotation2d(0_deg) });
    }
}

Trajectory::~Trajectory() { }

Trajectory::State Trajectory::sample(units::second_t time) const {
    decltype(states)::const_iterator upperBound = states.upper_bound(time),
                                     lowerBound = --states.lower_bound(time);
    
    bool noUpper = (upperBound == states.cend()),
         noLower = (lowerBound == states.cbegin());

    // No defined states D:
    if (noUpper && noLower) {
        return State{ 0_m, 0_m, 0_mps, frc::Rotation2d(0_deg) };
    }
    // Return the highest defined state if there is no upper bound.
    else if (noUpper) {
        State s(lowerBound->second);
        s.velocity = 0_mps;
        return s;
    }
    // Return the lowest defined state if there is no lower bound.
    else if (noLower) {
        State s(upperBound->second);
        s.velocity = 0_mps;
        return s;
    }

    auto [upperTime, upperState] = *upperBound;
    auto [lowerTime, lowerState] = *lowerBound;

    // Return the next state if is's close enough.
    if (units::math::abs(upperTime - lowerTime) < 0.03_s) {
        return upperState;
    }

    // Linear interpolation of all the values.

    double t = (time.value() - lowerTime.value()) / (upperTime.value() - lowerTime.value());

    units::meter_t xPos = ((upperState.xPos - lowerState.xPos) * t) + lowerState.xPos,
                   yPos = ((upperState.yPos - lowerState.yPos) * t) + lowerState.yPos;

    units::meters_per_second_t velocity = ((upperState.velocity - lowerState.velocity) * t) + lowerState.velocity;

    frc::Rotation2d rotation = ((upperState.rotation.Radians() - lowerState.rotation.Radians()) * t) + lowerState.rotation.Radians();

    return State{ xPos, yPos, velocity, rotation };
}

units::second_t Trajectory::getDuration() const {
    // Reverse iterator to get the last element.
    return states.rbegin()->first;
}

frc::Pose2d Trajectory::getInitialPose() const {
    const State& state(states.cbegin()->second);

    return frc::Pose2d(state.xPos, state.yPos, state.rotation);
}
