#pragma once

#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * This class is only to be used for ReveilLib simulations/testing.
 * @brief simulates a quadrature encoder with a customizable resolution to test multiple different types of encoders.
 */
class MockQuadEncoder : public ReadOnlyRotarySensor {
  public:
    MockQuadEncoder(int initial_reading, int resolution);
    double get_position() override; // Returns the distance in degrees the sensor has rotated
    int get_value(); // Returns the number of ticks the sensor has rotated
    void increment(); // Increases number of ticks the sensor has rotated
    void decrement(); // Decreases the number of ticks the sensor has rotated
  private:
    int value = 0;
    int resolution = 0;
};
} // namespace rev