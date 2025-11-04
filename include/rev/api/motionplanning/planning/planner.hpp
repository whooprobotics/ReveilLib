#pragma once

#include "rrt/rrt.hpp"
#include "spline/spline.hpp"
#include "gvf/gvf.hpp"
#include "../math/Pose2d.hpp"
#include <iostream>

class Planner {
public:
    RRT::Generator rrt;
    Spline globalPath;

    static constexpr double GOAL_BIAS = 0.4;
    static constexpr int RESOLUTION = 20;
    static constexpr double GOAL_RADIUS = 2;
    static constexpr size_t RRT_ITERATIONS = 2000;
    static constexpr double CONVERGENCE_FACTOR = 10;

    Planner(Pose2d startPose, Pose2d goalPose, RRT::BoundingBox bounds, std::vector<RRT::Obstacle>& obstacles) 
        : rrt(startPose.pos, goalPose.pos, bounds, RESOLUTION, GOAL_BIAS, GOAL_RADIUS, RRT_ITERATIONS, obstacles)
    {}

    void genGlobalPath();

    Eigen::Vector2d getDirectionVectorAt(Eigen::Vector2d point);
};