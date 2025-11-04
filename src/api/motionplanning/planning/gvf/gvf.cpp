#include "rev/api/motionplanning/planning/gvf/gvf.hpp"
#include <iostream>

Eigen::Vector2d GuidingVectorField::calculateVectorAt(Eigen::Vector2d point, const Spline& spline, double convergenceFactor) {
    double t = spline.nearestS(point, 1000);
    Eigen::Vector2d tangent = spline.calculateDerivativeAt(t);
    Eigen::Vector2d perpendicular = (spline.calculateAt(t) - point) * convergenceFactor;
    return (tangent + perpendicular).normalized();
}