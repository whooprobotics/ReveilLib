#include "rev/api/motionplanning/planning/rrt/rrt.hpp"
#include <algorithm>

using namespace RRT;

Vec2d Generator::genRandPoint() const {
    if (!foundPath)
        return sampleInBoundsWithBias(bounds, goal, goalBias, rand);
    return sampleInInformedEllipse(start, goal, bestPathDistance, bounds, goalBias, rand);
}

Node* Generator::nearestNode(const Vec2d& point) const {
    return nodeManager.nearestNeighbor(point);
}

void Generator::nodesInRadiusofPoint(std::vector<Node*>& nodeList, double radius, const Vec2d& point) const {
    nodeManager.radiusSearch(nodeList, radius, point);
}

Node* Generator::findBestParent(const std::vector<Node*>& nodeList, const Vec2d& point, Node* nearestNode) const {
    Node* bestParent = nearestNode;
    double bestCost = bestParent->cost + (point - bestParent->point).magnitude();

    for (const auto& node : nodeList) {
        double cost = node->cost + (point - node->point).magnitude();
        if (cost < bestCost) {
            bestParent = node;
            bestCost = cost;
        }
    }

    return bestParent;
}

bool Generator::pointIsValid(const Vec2d& p) const {
    for (const auto& obstacle : obstacles) {
        if (obstacle.pointInObstacle(p)) {
            return false;
        }
    }
    return true;
}

bool Generator::lineIsValid(const Vec2d& p1, const Vec2d& p2) const {
    for (const auto& obstacle : obstacles) {
        if (obstacle.lineInObstacle(p1, p2)) {
            return false;
        }
    }
    return true;
}

void Generator::iterate() {
    Vec2d randPoint = genRandPoint();

    // Nearest Node to Point
    Node* nearestNode = this->nearestNode(randPoint);

    // Normalize the point to the node
    if (randPoint == nearestNode->point) {
        return;
    }
    randPoint = randPoint.steerToward(nearestNode->point, stepSize);

    // Check Collision
    if (!pointIsValid(randPoint)) {
        return;
    }

    // Choose best parent
    double searchRadius = std::max(stepSize * 2, 
        2 * stepSize * std::sqrt(std::log(nodeManager.size) / nodeManager.size));
    std::vector<Node*> nearbyNodes;
    nodesInRadiusofPoint(nearbyNodes, searchRadius, randPoint);

    Node* bestParent = findBestParent(nearbyNodes, randPoint, nearestNode);

    // Check if line from best parent to new node intersects an obstacle
    if (!lineIsValid(bestParent->point, randPoint)) {
        return;
    }

    // Wire new node to the best parent
    std::unique_ptr<Node> newNodePtr = std::make_unique<Node>(bestParent, randPoint);
    Node* newNode = newNodePtr.get();
    bestParent->children.emplace_back(std::move(newNodePtr));
    newNode->parent = bestParent;
    newNode->calculateCost();

    nodeManager.addNode(newNode);

    // Rewire nearby nodes
    for (auto &node : nearbyNodes) {
        if (node->point == newNode->point) continue;
        
        if (node->cost > newNode->cost + (newNode->point - node->point).magnitude() && lineIsValid(node->point, newNode->point)) {
            if (node->parent) {
                auto nodePtr = std::find_if(node->parent->children.begin(), node->parent->children.end(),
                    [node](const std::unique_ptr<Node>& ptr) {
                        return ptr.get() == node;
                });

                newNode->children.push_back(std::move(*nodePtr));
                node->parent->children.erase(nodePtr);
                node->parent = newNode;
                node->calculateCost();
                node->propagateCost();
            }
            
        }
    }

    if ((goal - newNode->point).magnitude() < stepSize) {
        std::unique_ptr<Node> newGoalNodePtr = std::make_unique<Node>(newNode, goal);
        Node* newGoalNode = newGoalNodePtr.get();
        newNode->children.emplace_back(std::move(newGoalNodePtr));
        newGoalNode->parent = newNode;
        newGoalNode->calculateCost();

        nodeManager.addNode(newGoalNode);

        foundPath = true;
        if (newNode->cost < bestPathCost) {
            bestPathCost = newNode->cost;
            bestPathDistance = newNode->cumulativeDistance;
        }
    }
}

Node* Generator::optimalNodeNearGoal() const {
    std::vector<Node*> goalCandidates;
    nodesInRadiusofPoint(goalCandidates, goalRadius, goal);

    if (goalCandidates.size() == 0) {
        return nullptr;
    }

    Node* bestCandidate = goalCandidates[0];

    for (auto& node : goalCandidates) {
        if (node->cost + (goal - node->point).magnitude() < bestCandidate->cost) {
            bestCandidate = node;
        }
    }

    return bestCandidate;
}

std::vector<Node*> Generator::getOptimalPath() const {
    std::vector<Node*>optimalPath;
    Node* node = optimalNodeNearGoal();
    while (node->parent != nullptr) {
        optimalPath.push_back(node);
        node = node->parent;
    }
    optimalPath.push_back(node);
    std::reverse(optimalPath.begin(), optimalPath.end());
    return optimalPath;
}

int Generator::iterateUntilPathFound(int maxIter) {
    for (int i = 0; i < maxIter; i++) {
        iterate();

        if (foundPath) return i;
        i++;
    }

    return 0;
}

void Generator::iterateIterations(int iter) {
    for (int i = 0; i < iter; i++) {
        iterate();
    }
}