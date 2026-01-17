#pragma once

#include "rev/api/alg/reckless/path.hh"
#include "rev/api/hardware/chassis/chassis.hh"
#include "rev/api/hardware/sensors/distance_reset.hh"
#include "pros/distance.hpp"
#include <memory>

namespace rev {

class ResetOdometry {
  public:
    /** Wraps mik::distance sensors into a group. Two distance sensors perpendicular to each other should be enough,
     * As you can reset both coords at once if the sensors are facing two different walls.
     * Ex: A front or back sensor with a left or right sensor with the robot in a corner.
     */
    ResetOdometry(const std::vector<rev::DistanceReset>& distance_sensors, std::shared_ptr<rev::Odometry> odom);
    
    /** 
     * @brief Gets an X or Y position of the robot based on the heading and distance from a wall.
     * The distance sensor must be facing the desired wall with no obstruction in order to work.
     * Choosing to reset off a top or bottom wall will reset the robots y positon, and a left or right
     * wall resetting the robots x position.
     * 
     * @param sensor_pos The side of the robot that the distance sensor is mounted.
     * @param wall_pos The wall that is being looked at by desired distance sensor.
     * @param max_reset_distance The maxiumum allowed of distance in inches that an odom axis can be changed.
     * 
     * @return A new x or y coordinate based on the wall desired sensor is faced at.
     */
    float get_reset_axis_pos(rev::DistancePosition sensor_pos, rev::WallPosition wall_pos, float angle);
    
    /** 
     * @brief Resets an X or Y position of the robot based on the heading and distance from a wall.
     * The distance sensor must be facing the desired wall with no obstruction in order to work.
     * Choosing to reset off a top or bottom wall will reset the robots y positon, and a left or right
     * wall resetting the robots x position.
     * 
     * @param sensor_pos The side of the robot that the distance sensor is mounted.
     * @param wall_pos The wall that is being looked at by desired distance sensor.
     * @param max_reset_distance The maxiumum allowed of distance in inches that an odom axis can be changed.
     * 
     * @return True if the desired axis was reset successfully.
     */
    bool reset_axis(rev::DistancePosition sensor_pos, rev::WallPosition wall_pos, float max_reset_distance);

    /** @returns vector containing all rev::DistanceReset sensors. */
    std::vector<rev::DistanceReset>& get_distance_sensors();
    
  private:
    float to_sensor_offset_constant(rev::DistancePosition sensor_pos);
    float to_wall_pos_constant(rev::WallPosition wall_pos);
    float to_wall_angle_constant(rev::WallPosition wall_pos);

    std::vector<rev::DistanceReset> distance_sensors;
    std::shared_ptr<rev::Odometry> odom;
    
};
} // namespace rev