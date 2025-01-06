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
     */
    PurePursuitSegment(
        std::shared_ptr<Motion> imotion,
        std::shared_ptr<Correction> icorrection,
        std::shared_ptr<Stop> istop,
        QLength look_ahead_distance,
        QLength wheelbase,
        double ikp,
        double iki,
        double ikd            
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

    std::vector<PointVector> path_waypoints;
    Pose start_point;
    PointVector last_point;
    size_t current_idx;

private:
    std::shared_ptr<Motion> motion;
    std::shared_ptr<Correction> correction;
    std::shared_ptr<Stop> stop; 

    QLength look_ahead_distance;
    QLength wheelbase;
    
    QLength drop_early = 0_in;

    stop_state new_state;

    double kp;
    double ki;
    double kd;

    double integral = 0;
    double prev_error = 0;

    std::tuple<double, double> pows;
    std::tuple<double, double> corrected_pows;

    SegmentStatus last_status{SegmentStatus::drive(0, 0)};

    // Helper methods
    /**
     * @brief Find the target point on the path at the look-ahead distance
     * 
     * @param current_state Current odometry state of the robot
     * @return PointVector The target point for Pure Pursuit
     */
    PointVector find_target_point(OdometryState current_state);

    /**
     * @brief Calculate the remaining distance to the end of the path
     * 
     * @param current_state Current odometry state of the robot
     * @return double Remaining distance in meters
     */
    QLength calculate_remaining_distance(OdometryState current_state);

    /**
     * @brief PID controller for Pure Pursuit
     * 
     * @param current_state Current odometry state of the robot
     * @return std::tuple<double, double> Tuple of left and right motor powers
     */
    std::tuple<double, double> PID(OdometryState current_state, double base_power);
};

} // namespace rev