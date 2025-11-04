#include "rev/api/motionplanning/planning/rrt/node.hpp"

using namespace RRT;

#include <iostream>

double Node::calculateCost() {
    if (parent == nullptr) {
        return 0;
    }

    Vec2d v1, v2;
    if (parent->parent == nullptr) {
        v1 = Vec2d(0,0);
        v2 = Vec2d(0,0);
    } else {
        v1 = parent->point - parent->parent->point;
        v2 = point - parent->point;
    }

    double angleCost = 0.0;
    if (v1.magnitude() > 1e-6 && v2.magnitude() > 1e-6) {
        double cosTheta = v1.dot(v2) / (v1.magnitude() * v2.magnitude());
        if (cosTheta > 1.0) cosTheta = 1.0;
        if (cosTheta < -1.0) cosTheta = -1.0;
        angleCost = angleWeight * (1 - cosTheta);
    }

    cumulativeDistance = parent->cumulativeDistance + (point - parent->point).magnitude();

    double newCost = angleCost + parent->cost + (point - parent->point).magnitude();
    cost = newCost;
    return newCost;
}

void Node::propagateCost() {
    for (auto &child : children) {
        child->calculateCost();
        child->propagateCost();
    }
}