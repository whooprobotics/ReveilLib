#include "rev/api/motionplanning/planning/planner.hpp"

void Planner::genGlobalPath() {
    std::cout << "Starting iterations" << std::endl;
    rrt.iterateIterations(RRT_ITERATIONS);
    std::cout << "Done iterating" << std::endl;

    auto v = rrt.getOptimalPath();
    globalPath = Spline(v, 3);
    
    std::cout << "Generating spline" << std::endl;
    globalPath.generate(1e-5);
}

Eigen::Vector2d Planner::getDirectionVectorAt(Eigen::Vector2d point) {
    return GuidingVectorField::calculateVectorAt(point, globalPath, CONVERGENCE_FACTOR);
}