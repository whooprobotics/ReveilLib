#pragma once
#include "odometry.hh"
#include "pros/rtos.hpp"
#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/devices/optical/otos.hh"
#include <vector>
#include <string>
#include <sstream>

namespace rev {
/**
 * @brief Odometry implementation using optical odometry sensor
 * 
 */
class OpticalOdometry : public Odometry, public AsyncRunnable {
  public:
    /**
     * @brief Get the current position
     * 
     * The implementation of this is thread-safe
     * 
     * @return OdometryState
     */
    OdometryState get_state() override;
    void set_position(Position pos) override;
    void reset_position() override;
    void step() override;

    OpticalOdometry(std::shared_ptr<OTOS> sensor, QLength ilongitudinal_offset, QLength ilateral_offset);

  private:
    std::shared_ptr<OTOS> optical_sensor; // Tracking sensor
    pros::Mutex current_position_mutex;

    OdometryState current_position{{0_in, 0_in, 0_deg}, {0_mps, 0_mps, 0_deg / second}};

    // Offset of the longitudinal wheel to the right of the center of the robot
    QLength longitudinal_offset;
    // Likewise, for the lateral wheel backward from the center of rotation
    QLength lateral_offset;
};

} // namespace rev