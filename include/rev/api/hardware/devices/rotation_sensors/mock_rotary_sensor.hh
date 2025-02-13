#pragma once

#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * This class is only to be used for ReveilLib simulations/testing.
 * @brief simulates a Vex rotation sensor
 */
class MockRotarySensor : public ReadOnlyRotarySensor {
  public:
    MockRotarySensor(int reading);
    double get_position() override; // Returns distance in degrees the sensor has rotated
    int get_value(); // Returns number of ticks rotated
    void increment(); // Increases number of ticks
    void decrement(); // Decreases number of ticks
  private:
    int value = 0;
};

} // namespace rev