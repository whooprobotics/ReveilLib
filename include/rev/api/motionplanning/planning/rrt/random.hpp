#pragma once

#include <random>
#include <cmath>
#include "rev/api/motionplanning/math/Vec2d.hpp"

namespace RRT {
    struct BoundingBox {
        double minX;
        double minY;
        double maxX;
        double maxY;

        BoundingBox(double minX, double minY, double maxX, double maxY)
            : minX(minX), minY(minY), maxX(maxX), maxY(maxY)
        {}
    };

    constexpr double PI = 3.14159265358979323846;

    struct Random {
        std::mt19937_64 gen;
        std::uniform_real_distribution<double> unif{0.0, 1.0};
        std::uniform_real_distribution<double> unifAngle{0.0, 2.0*PI}; // radians

        Random(uint64_t seed = std::random_device{}()) : gen(seed) {}

        double rand() { return unif(gen); }
        double randAngle() { return unifAngle(gen); }
    };

    Vec2d sampleInBoundsWithBias(BoundingBox boundingBox, const Vec2d& goal, double goalBias, Random& rng);

    Vec2d sampleInInformedEllipse(const Vec2d& start, const Vec2d& goal, double cBest, BoundingBox boundingBox, double goalBias, Random& rng);
}