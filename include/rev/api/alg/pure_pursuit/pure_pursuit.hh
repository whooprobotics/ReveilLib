#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <cmath>

#include "rev/api/alg/reckless/path.hh"
#include "rev/api/async/async_awaitable.hh"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/chassis.hh"

namespace rev {

/**
 * @brief Pure Pursuit Path Segment
 *
 * Implements the Pure Pursuit algorithm for path tracking.
 */
class PurePursuitSegment : public RecklessSegment {
public:
    /**
     * @brief Construct a new Pure Pursuit Segment
     * 
     * @param imotion Shared pointer to Motion controller
     * @param icorrection Shared pointer to Correction mechanism
     * @param istop Shared pointer to Stop mechanism
     * @param path_points Vector of waypoints defining the path
     * @param look_ahead_distance Look-ahead distance (L_d) for Pure Pursuit
     * @param wheelbase Wheelbase of the robot
     * @param tolerance Lateral tolerance for path tracking
     */
    PurePursuitSegment(
        std::shared_ptr<Motion> imotion,
        std::shared_ptr<Correction> icorrection,
        std::shared_ptr<Stop> istop,
        std::vector<PointVector> path_points,
        QLength look_ahead_distance = 1.0_ft,
        QLength wheelbase = 0.5_ft,             
        QLength tolerance = 1_in              
    );

    /**
     * @brief Initialize the Pure Pursuit segment
     * 
     * @param initial_state The initial odometry state of the robot
     */
    void init(OdometryState initial_state) override;

    /**
     * @brief Compute the next step in the Pure Pursuit segment
     * 
     * @param current_state The current odometry state of the robot
     * @return SegmentStatus The status after computing control commands
     */
    SegmentStatus step(OdometryState current_state) override;

    /**
     * @brief Clean-up after completing the segment
     */
    void clean_up() override;

protected:
    /**
     * @brief Set the path waypoints
     * 
     * @param path_waypoints Vector of waypoints defining the path
     */
    void set_path(std::vector<PointVector> path_waypoints);

    /**
     * @brief Get the generated waypoints
     * 
     * @return std::vector<PointVector> Vector of waypoints
     */
    virtual std::vector<PointVector> generate_waypoints() = 0;

private:
    std::shared_ptr<Motion> motion;
    std::shared_ptr<Correction> correction;
    std::shared_ptr<Stop> stop;

    std::vector<PointVector> path_points;
    std::vector<PointVector> path_waypoints;

    QLength look_ahead_distance;
    QLength wheelbase;
    QLength tolerance;

    size_t current_idx;

    // Helper methods
    /**
     * @brief Find the target point on the path at the look-ahead distance
     * 
     * @param current_state Current odometry state of the robot
     * @return PointVector The target point for Pure Pursuit
     */
    PointVector findTargetPoint(const OdometryState& current_state);

    /**
     * @brief Calculate the remaining distance to the end of the path
     * 
     * @param current_state Current odometry state of the robot
     * @return double Remaining distance in meters
     */
    QLength calculateRemainingDistance(const OdometryState& current_state);
};

} // namespace rev