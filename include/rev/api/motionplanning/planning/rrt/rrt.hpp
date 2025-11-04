#pragma once 

#include <cmath>
#include <memory>
#include <vector>
#include <random>

#include "obstacle.hpp"
#include "node.hpp"
#include "nodeManager.hpp"
#include "random.hpp"

#include "rev/api/motionplanning/math/Vec2d.hpp"

namespace RRT {
    class Generator {
    public:
        Generator(Vec2d start, Vec2d goal, BoundingBox bounds, double stepSize, double goalBias, double goalRadius, int iterations, const std::vector<Obstacle>& obstacles)
            :   start(start), 
                goal(goal), 
                stepSize(stepSize), 
                bounds(bounds), 
                goalBias(goalBias), 
                goalRadius(goalRadius), 
                rand(), 
                obstacles(obstacles),
                root(std::make_unique<Node>(nullptr, start)),
                foundPath(false),
                bestPathCost(std::numeric_limits<double>::infinity()),
                bestPathDistance(std::numeric_limits<double>::infinity()),
                nodeManager(iterations)
        {
            root = std::make_unique<Node>(nullptr, start);
            nodeManager.addNode(root.get());
        }

        void iterate();
        int iterateUntilPathFound(int maxIter);
        void iterateIterations(int iter);
        Node* optimalNodeNearGoal() const;
        std::vector<Node*> getOptimalPath() const;
        std::vector<Obstacle> obstacles;
        NodeManager nodeManager;

    private:
        Vec2d start;
        Vec2d goal;
        BoundingBox bounds;
        std::unique_ptr<Node> root;

        mutable Random rand;

        double stepSize;
        double goalBias;
        double goalRadius;

        bool foundPath;
        double bestPathCost;
        double bestPathDistance;

        Node* findBestParent(const std::vector<Node*>& nodeList, const Vec2d& point, Node* nearestNode) const;
        void nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Vec2d& point) const;
        Vec2d genRandPoint() const;
        bool pointIsValid(const Vec2d& p) const;
        bool lineIsValid(const Vec2d& p1, const Vec2d& p2) const;
        Node* nearestNode(const Vec2d& point) const;
    };
};