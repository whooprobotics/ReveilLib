#pragma once

#include "rev/api/hardware/devices/rotation_sensors/rotary_sensors.hh"

namespace rev {
/**
 * @brief Simulated implementation of the RotationSensor class
 * 
 */
class MockRotationSensor : public RotarySensor {
 public:
  /**
   * @brief Constructs a new MockRotationSensor simulated device
   * 
   * @param reading Initial sensor reading of the encoder
   */
  MockRotationSensor(int reading);

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
   * @brief Gets the number of ticks rotated from the zero position
   * 
   * @returns Ticks rotated from zero
   */
  int get_value();

  /**
   * @brief Increments the tick count by 1
   * 
   */
  void increment();

  /**
   * @brief Decrements the tick count by 1
   */
  void decrement();

 private:
  int value = 0;
};

} // namespace rev