#include <Trajectory/TrajectoryRecorder.h>
#include <fstream>
#include <iostream>

TrajectoryRecorder::TrajectoryRecorder()
: lastStateIt(states.cend()) { }

TrajectoryRecorder::~TrajectoryRecorder() = default;

void TrajectoryRecorder::clear() {
    states.clear();
    lastStateIt = states.cend();
}

void TrajectoryRecorder::writeToCSV(std::filesystem::path path) {
    std::ofstream file(path);
    file.clear();

    file << "time,x_pos,y_pos,velocity,rotation,action\n";

    for (const auto& [time, state] : states) {
        file << time.value() << ','
        << state.xPos.value() << ','
        << state.yPos.value() << ','
        << state.velocity.value() << ','
        << state.rotation.Radians().value() << ','
        << "0\n";
    }
}

void TrajectoryRecorder::addState(units::second_t dt, frc::Pose2d pose) {
    if (dt == 0_s) return;

    units::second_t time(dt);
    units::meters_per_second_t vel(0_mps);

    if (lastStateIt != states.cend()) {
        const decltype(states)::value_type& lastState(*lastStateIt);

        time += lastState.first;

        // The component changes in position.
        units::meter_t dx(pose.X() - lastState.second.xPos),
                       dy(pose.Y() - lastState.second.yPos);

        // The change in position.
        units::meter_t dd(units::math::hypot(dx, dy));

        // The velocity (d/t).
        vel = dd / dt;
    }

    lastStateIt = states.emplace_hint(states.cend(),
        time, Trajectory::State{ pose.X(), pose.Y(), vel, pose.Rotation() }
    );
}
