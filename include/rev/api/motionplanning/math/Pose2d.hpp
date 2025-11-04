#pragma once

#include "Vec2d.hpp"

struct Pose2d {
    Vec2d pos;
    double heading;
    Vec2d vel;
    Vec2d acc;

    Pose2d(double x, double y) 
        : pos(x, y), vel(x, y), acc(x, y)
    {}

    Pose2d() = default;
};