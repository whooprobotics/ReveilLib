#include "rev/api/alg/pure_pursuit/pure_pursuit.hh"

namespace rev {

PurePursuitSegment::PurePursuitSegment(std::shared_ptr<Motion> imotion,
                                       std::shared_ptr<Correction> icorrection,
                                       std::shared_ptr<Stop> istop,
                                       std::vector<PointVector> path_points,
                                       QLength look_ahead_distance,
                                       QLength wheelbase,
                                       QLength tolerance):
    motion(imotion),
    correction(icorrection),
    stop(istop),
    path_points(path_points),
    look_ahead_distance(look_ahead_distance),
    wheelbase(wheelbase),
    tolerance(tolerance),
    current_idx(0)
    {}

PurePursuitSegment::~PurePursuitSegment() {
}

PointVector PurePursuitSegment::findTargetPoint(const OdometryState& current_state) {
    QLength min_distance = 100_ft; // Initialize with a large value
    size_t target_idx = current_idx;
    PointVector target_point = path_points.back(); // Default to the last waypoint

    // Iterate through the path points starting from the current index
    for (size_t i = current_idx; i < path_points.size(); ++i) {
        QLength dx = path_points[i].x - current_state.pos.x;
        QLength dy = path_points[i].y - current_state.pos.y;
        QLength distance = sqrt(square(dx) + square(dy)); // Assuming QLength can handle sqrt and conversion

        if (distance >= look_ahead_distance) {
            target_idx = i;
            target_point = path_points[i];
            break;
        }
    }

    // Update the current index
    current_idx = target_idx;

    std::cout << "PurePursuitSegment: Target Point - (" << target_point.x.convert(foot) << " ft, " << target_point.y.convert(foot) << " ft)" << std::endl;

    return target_point;
}

// Helper method to calculate the remaining distance to the end of the path
QLength PurePursuitSegment::calculateRemainingDistance(const OdometryState& current_state) {
    QLength dx = path_points.back().x - current_state.pos.x;
    QLength dy = path_points.back().y - current_state.pos.y;
    QLength distance = sqrt(square(dx) + square(dy)); // Assuming QLength can handle sqrt and conversion
    return distance;
}

void PurePursuitSegment::init(OdometryState initial_state) {
    // Reset the current index to the starting point
    current_idx = 0;
}

// Step function implementation
SegmentStatus PurePursuitSegment::step(OdometryState current_state) {
    // Find the target point at the look-ahead distance
    PointVector target = findTargetPoint(current_state);

    // If current_idx is out of bounds, stop the robot
    if (current_idx >= path_points.size()) {
        std::cout << "PurePursuitSegment: Reached end of path. Initiating brake." << std::endl;
        return SegmentStatus::brake();
    }

    std::tuple<double, double> pows = motion->gen_powers(
        current_state,
        {target.x, target.y, 0_deg},
        {path_points[0].x, path_points[0].y, 0_deg},
        0.0_ft
    );

    // Apply correction to the motor powers
    std::tuple<double, double> corrected_pows = correction->apply_correction(
        current_state,
        {target.x, target.y, 0_deg},
        {path_points[0].x, path_points[0].y, 0_deg},
        0.0_ft,
        pows
    );

    // Check if the robot is within the tolerance of the target point
    Pose error = current_state.pos.to_relative({target.x, target.y, 0_deg});
    QLength distance = sqrt(square(error.x) + square(error.y));

    if (distance < tolerance) {
        std::cout << "PurePursuitSegment: Within tolerance of target point. Advancing to next waypoint." << std::endl;
        ++current_idx;
        // Check if the next index is beyond the path
        if (current_idx >= path_points.size()) {
            std::cout << "PurePursuitSegment: No more waypoints. Initiating brake." << std::endl;
            return SegmentStatus::brake();
        }
    }

    return SegmentStatus::drive(corrected_pows);
}

void PurePursuitSegment::clean_up() {
    // Clean-up implementation (if needed)
}

} // namespace rev