#pragma once

#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * @brief Simulated implementation of the QuadEncoder class
 * 
 */
class MockQuadEncoder : public RotarySensor {
 public:
  /**
   * @brief Constructs a new MockQuadEncoder simulated device
   * 
   * @param initial_reading Initial sensor reading of the encoder
   */
  MockQuadEncoder(int initial_reading);
  
  /**
   * @brief Gets the current position of the sensor
   * 
   * @returns Degrees rotated from the zero position
   */
  double get_position() override;

  /**
   * @brief Verifies that the sensor ports are properly connected
   * 
   * @returns Zeros since it's a simulated device
   */
  std::pair<std::uint8_t, std::uint8_t> check_ports() override;

  /**
   * @brief Gets the number of encoder ticks rotated
   * 
   * @returns Tick count (1 rotation = 8192 ticks)
   */
  int get_value();

  /**
   * @brief Increments the tick counter by 1
   */
  void increment();

  /**
   * @brief Decrements the tick counter by 1
   */
  void decrement();

  /**
   * @brief Gets the number of times the encoder has looped around to 0
   * 
   * @returns Looparound count
   */
  int get_looparounds();

 private:
  int value = 0;
  int looparounds = 0;
};
}  // namespace rev