#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"
#include "rev/api/hardware/devices/rotation_sensors/mock_rotary_sensor.hh"
#include "rev/api/hardware/devices/rotation_sensors/mock_quad_encoder.hh"
#include "rev/api/hardware/devices/gyroscope/mock_imu.hh"
#include "rev/rev.hh"

using namespace rev;
using std::cout, std::endl;

void quadencodertest() {  
  std::shared_ptr<MockQuadEncoder> forward = std::make_shared<MockQuadEncoder>(0);
  std::shared_ptr<MockQuadEncoder> lateral = std::make_shared<MockQuadEncoder>(0);
  rev::MockImu imu(0);

  // TwoRotationInertialOdometry(forward, lateral, imu);

  std::shared_ptr<AsyncRunner> odomRunner();

  for(int i = 0; i < 100; i++) { forward->increment(); }

  for(int i = 0; i < 900; i++) { lateral->increment(); }

  for(int i = 0; i < 200; i++) { forward->decrement(); }

  for(int i = 0; i < 1100; i++) { lateral->decrement(); }
}

void rotationsensortest() {
  MockRotarySensor forward(0);
  MockRotarySensor lateral(0);

  for(int i = 0; i < 100; i++) {
    forward.increment();
  }

  ASSERT_NEAR(forward.get_value(), 100, 1);
  ASSERT_NEAR(forward.get_position(), 1, 1.0/36000.0);

  for(int i = 0; i < 900; i++) {
    lateral.increment();
  }

  ASSERT_NEAR(lateral.get_value(), 900, 1);
  ASSERT_NEAR(lateral.get_position(), 9, 1.0/36000.0);

  for(int i = 0; i < 200; i++) {
    forward.decrement();
  }

  ASSERT_NEAR(forward.get_value(), 35900, 1);
  ASSERT_NEAR(forward.get_position(), 359, 1.0/36000.0);

  for(int i = 0; i < 1100; i++) {
    lateral.decrement();
  }

  ASSERT_NEAR(lateral.get_value(), 35800, 1);
  ASSERT_NEAR(lateral.get_position(), 358, 1.0/36000.0);
}

TEST(OdomTests, QuadEncoderTest) {
  quadencodertest();
}

// TEST(OdomTests, RotarySensorTest) {
//   rotationsensortest();
// }