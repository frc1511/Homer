#include <Trajectory/TrajectoryRecorder.h>
#include <fstream>

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
        << state.pose.X().value() << ','
        << state.pose.Y().value() << ','
        << state.velocity.value() << ','
        << state.pose.Rotation().Radians().value() << ','
        << "0\n";
    }
}

void TrajectoryRecorder::addState(units::second_t dt, frc::Pose2d pose) {
    if (dt == 0_s) return;

    units::second_t time(dt);
    units::meters_per_second_t vel(0_mps);

    if (lastStateIt != states.cend()) {
        const auto& [t, state] = *lastStateIt;
        time += t;

        // The change in position.
        frc::Twist2d twist(state.pose.Log(pose));

        // The change in position.
        units::meter_t dd(units::math::hypot(twist.dx, twist.dy));

        // The velocity (d/t).
        vel = dd / dt;
    }

    lastStateIt = states.emplace_hint(states.cend(),
        time, Trajectory::State{ pose, vel }
    );
}
