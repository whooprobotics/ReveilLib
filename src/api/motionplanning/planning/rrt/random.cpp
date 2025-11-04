#include "rev/api/motionplanning/planning/rrt/random.hpp"

Vec2d RRT::sampleInBoundsWithBias(BoundingBox boundingBox, const Vec2d& goal, double goalBias, Random& rng) {
    if (rng.rand() < 0.5) {
        return goal;
    }

    double x = boundingBox.minX + rng.rand() * std::abs(boundingBox.maxX - boundingBox.minX);
    double y = boundingBox.minY + rng.rand() * std::abs(boundingBox.maxY - boundingBox.minY);
    return Vec2d(x, y);
}

Vec2d RRT::sampleInInformedEllipse(const Vec2d& start, const Vec2d& goal, double cBest, BoundingBox boundingBox, double goalBias, Random& rng) {
    Vec2d d = goal - start;
    double cMin = d.magnitude();
    if (cBest <= cMin) return sampleInBoundsWithBias(boundingBox, goal, goalBias, rng);

    double a = cBest * 0.5;
    double b = 0.5 * std::sqrt(cBest*cBest - cMin*cMin);

    Vec2d center = (start + goal) * 0.5;

    double angle = std::atan2(d.y(), d.x());
    double ca = std::cos(angle);
    double sa = std::sin(angle);

    double r = std::sqrt(rng.rand());
    double theta = rng.randAngle();

    double xball = r * std::cos(theta);
    double yball = r * std::sin(theta);

    double xs = a * xball;
    double ys = a * yball;

    return Vec2d(ca * xs - sa * ys + center.x(), sa * xs + ca * ys + center.y());
}