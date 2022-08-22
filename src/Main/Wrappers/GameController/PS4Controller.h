#pragma once

class ThunderPS4Controller {
public:
    enum Button {
        TRIANGLE = 4,
        CIRCLE = 3,
        CROSS = 2,
        SQUARE = 1,

        Y = TRIANGLE,
        B = CIRCLE,
        A = CROSS,
        X = SQUARE,

        LEFT_BUMPER = 5,
        RIGHT_BUMPER = 6,

        SHARE = 9,
        OPTIONS = 10,

        BACK = SHARE,
        START = OPTIONS,

        LEFT_STICK = 11,
        RIGHT_STICK = 12,
    };

    enum Axis {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 2,
        RIGHT_Y = 5,

        LEFT_TRIGGER = 3,
        RIGHT_TRIGGER = 4,
    };
};
