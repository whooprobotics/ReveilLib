#include "rev/api/motionplanning/planning/rrt/obstacle.hpp"

using namespace RRT;

static int orientation(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
    double val = (b.y() - a.y()) * (c.x() - b.x()) - (b.x() - a.x()) * (c.y() - b.y());
    if (std::fabs(val) < 1e-9) return 0;
    return (val > 0) ? 1 : 2;
}

static bool onSegment(const Vec2d& p, const Vec2d& q, const Vec2d& r) {
    return (q.x() <= std::max(p.x(), r.x()) + 1e-9 &&
            q.x() >= std::min(p.x(), r.x()) - 1e-9 &&
            q.y() <= std::max(p.y(), r.y()) + 1e-9 &&
            q.y() >= std::min(p.y(), r.y()) - 1e-9);
}

static bool segmentsIntersect(const Vec2d& p1, const Vec2d& p2, const Vec2d& q1, const Vec2d& q2) {
    int o1 = orientation(p1, p2, q1);
    int o2 = orientation(p1, p2, q2);
    int o3 = orientation(q1, q2, p1);
    int o4 = orientation(q1, q2, p2);

    if (o1 != o2 && o3 != o4)
        return true;

    if (o1 == 0 && onSegment(p1, q1, p2)) return true;
    if (o2 == 0 && onSegment(p1, q2, p2)) return true;
    if (o3 == 0 && onSegment(q1, p1, q2)) return true;
    if (o4 == 0 && onSegment(q1, p2, q2)) return true;

    return false;
}

bool Obstacle::pointInObstacle(const Vec2d& p) const {
    bool inside = false;
    int n = polygon.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Vec2d& pi = polygon[i];
        const Vec2d& pj = polygon[j];

        bool intersect = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
                         (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y()) + pi.x());
        if (intersect)
            inside = !inside;
    }
    return inside;
}

bool Obstacle::lineInObstacle(const Vec2d& p1, const Vec2d& p2) const {
    // Case 1: either endpoint inside polygon
    if (pointInObstacle(p1) || pointInObstacle(p2))
        return true;

    // Case 2: check intersection with each polygon edge
    int n = polygon.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Vec2d& a = polygon[j];
        const Vec2d& b = polygon[i];
        if (segmentsIntersect(p1, p2, a, b))
            return true;
    }

    // No collision found
    return false;
}