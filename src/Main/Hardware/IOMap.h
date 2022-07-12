#pragma once

/*
  ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
  ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
  ██║██║   ██║    ██╔████╔██║███████║██████╔╝
  ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
  ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
  ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2022
*/

// #define TEST_BOARD

#ifdef TEST_BOARD
#   include <Hardware/_TestBoardIOMap.h>
#else
#   include <Hardware/_MainIOMap.h>
#endif

// CAN

#define CAN_SWERVE_DRIVE_MOTOR_FL 6
#define CAN_SWERVE_DRIVE_MOTOR_BL 4
#define CAN_SWERVE_DRIVE_MOTOR_BR 7
#define CAN_SWERVE_DRIVE_MOTOR_FR 5

#define CAN_SWERVE_ROT_MOTOR_FL 2
#define CAN_SWERVE_ROT_MOTOR_BL 8
#define CAN_SWERVE_ROT_MOTOR_BR 3
#define CAN_SWERVE_ROT_MOTOR_FR 1

#define CAN_SWERVE_CAN_CODER_FL 11
#define CAN_SWERVE_CAN_CODER_BL 10
#define CAN_SWERVE_CAN_CODER_BR 13
#define CAN_SWERVE_CAN_CODER_FR 12

// PWM

#define PWM_BLINKY_BLINKY 0