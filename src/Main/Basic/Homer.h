#pragma once

#include <frc/TimedRobot.h>

#include <Basic/Mechanism.h>
#include <Control/Controls.h>
#include <Vision/Limelight.h>
#include <Vision/Camera.h>
#include <RollingRaspberry/RollingRaspberry.h>
#include <Drive/Drive.h>
#include <Illumination/BlinkyBlinky.h>
#include <Autonomous/Autonomous.h>
#include <Hardware/IOMap.h>
#include <vector>

class Homer : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

private:
    void reset(Mechanism::MatchMode mode);

    RollingRaspberry rollingRaspberry;
    Limelight limelight;
    
    Drive drive { &limelight, &rollingRaspberry };
    Autonomous autonomous { &drive };

#ifndef TEST_BOARD
    BlinkyBlinky blinkyBlinky;
#endif

    Controls controls { &drive, &limelight,
#ifndef TEST_BOARD
        &blinkyBlinky,
#endif
    };

    std::vector<Mechanism*> allMechanisms {
        &drive, &controls, &autonomous, &limelight, &rollingRaspberry,
#ifndef TEST_BOARD
        &blinkyBlinky,
#endif
    };

    std::vector<Mechanism*> universalMechanisms {
        &limelight, &rollingRaspberry,
#ifndef TEST_BOARD
        &blinkyBlinky,
#endif
    };
};
