#pragma once

class ThunderXboxController {
public:
    enum Button {
        Y = 4,
        B = 2,
        A = 1,
        X = 3,

        TRIANGLE = Y,
        CIRCLE = B,
        CROSS = A,
        SQUARE = X,

        LEFT_BUMPER = 5,
        RIGHT_BUMPER = 6,

        BACK = 7,
        START = 8,

        SHARE = BACK,
        OPTIONS = START,

        LEFT_STICK = 11,
        RIGHT_STICK = 12,
    };

    enum Axis {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 4,
        RIGHT_Y = 5,

        LEFT_TRIGGER = 2,
        RIGHT_TRIGGER = 3,
    };
};
