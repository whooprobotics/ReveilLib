#pragma once

#include <vector>

#include "rev/api/motionplanning/math/Vec2d.hpp"

namespace RRT {
    struct Obstacle {
        std::vector<Vec2d> polygon;

        // Clockwise winding polygon
        Obstacle(const std::vector<Vec2d>& polygon)
            : polygon(polygon)
        {}

        static Obstacle fromRectVertices(double leftX, double bottomY, double rightX, double topY) {
            return Obstacle(
                std::vector<Vec2d>{
                    Vec2d(leftX, bottomY),
                    Vec2d(rightX, bottomY),
                    Vec2d(rightX, topY),
                    Vec2d(leftX, topY)
                }
            );
        }

        static Obstacle fromRectBottomLeft(double blX, double blY, double width, double height) {
            return Obstacle(
                std::vector<Vec2d>{
                    Vec2d(blX, blY),
                    Vec2d(blX + width, blY),
                    Vec2d(blX + width, blY + height),
                    Vec2d(blX, blY + height)
                }
            );
        }

        bool pointInObstacle(const Vec2d& p) const;
        bool lineInObstacle(const Vec2d& p1, const Vec2d& p2) const;
    };
}