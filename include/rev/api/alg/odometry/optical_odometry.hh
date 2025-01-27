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
    /**
     * @brief Set the current position. Allows user to use absolute field coordinates
     */
    void set_position(Position pos) override;
    /**
     * @brief Reset the current position to {0, 0, 0}
     */
    void reset_position() override;
    /**
     * @brief Updates the odometry values
     */
    void step() override;

    OpticalOdometry(std::shared_ptr<OTOS> sensor, QLength ilongitudinal_offset, QLength ilateral_offset);

  private:
    std::shared_ptr<OTOS> optical_sensor; // Tracking sensor
    pros::Mutex current_position_mutex;

    OdometryState current_position{{0_in, 0_in, 0_deg}, {0_mps, 0_mps, 0_deg / second}};
    OdometryState sensor_reading{{0_in, 0_in, 0_deg}, {0_mps, 0_mps, 0_deg / second}};
    OdometryState last_sensor_reading{{0_in, 0_in, 0_deg}, {0_mps, 0_mps, 0_deg / second}};

    QLength x_diff = 0_in;
    QLength y_diff = 0_in;
    QAngle h_diff = 0_deg;
};

} // namespace rev
