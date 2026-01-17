#pragma once

#include "pros/distance.hpp"  
#include <string>

namespace rev {

enum class DistancePosition { FRONT_SENSOR, REAR_SENSOR, LEFT_SENSOR, RIGHT_SENSOR };
enum class WallPosition { TOP_WALL, BOTTOM_WALL, LEFT_WALL, RIGHT_WALL };

class DistanceReset : public pros::Distance {

public:
    /** 
     * @brief Creates a new distance sensor object that contains its position and offset from tracking center, on the port specified.
     * @param index The port index for this sensor. The index is zero-based.
     * @param position The face of the robot that the distance sensor is mounted to.
     * @param x_center_offset The horizontal offset from the tracking center, right is +, left is -
     * @param y_center_offset The vertical offset from the tracking center, forward is +, backwards is -
     */
    DistanceReset(int port, DistancePosition position, float x_center_offset, float y_center_offset);

    /** @return The position of the distance sensor. */
    DistancePosition position() const;

    /** @return The horizontal offset */
    float x_center_offset() const;

    /** @return The vertical offset */
    float y_center_offset() const;

    /** @return Sets the vertical offset */
    void x_center_offset(float new_offset);

    /** @return Sets the horizontal offset */
    void y_center_offset(float new_offset);
    
private:
    std::string to_sensor_name(DistancePosition sensor_pos);
        
    int port_;
    DistancePosition position_;
    float x_center_offset_;
    float y_center_offset_;
    std::string name_;
};
} // namespace rev