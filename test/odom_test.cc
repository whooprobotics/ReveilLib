#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/alg/odometry/two_rotation_inertial_odometry.hh"
#include "rev/api/hardware/devices/gyroscope/mock_imu.hh"
#include "rev/api/hardware/devices/rotation_sensors/mock_quad_encoder.hh"
#include "rev/api/hardware/devices/rotation_sensors/mock_rotary_sensor.hh"
#include "rev/rev.hh"

using namespace rev;
using std::cout, std::endl;

static void printOdomState(OdometryState state,
                           std::shared_ptr<ReadOnlyRotarySensor> forward,
                           std::shared_ptr<ReadOnlyRotarySensor> lateral) {
  cout << "x: " << state.pos.x.convert(inch) << endl
       << "y: " << state.pos.y.convert(inch) << endl;
  cout << "Forward value: " << forward->get_position()
       << "\tForward angle: " << forward->get_position() << endl;
  cout << "Lateral value: " << lateral->get_position()
       << "\tLateral angle: " << lateral->get_position() << endl;
}

void quadencodertest(QLength wheel_diameter) {
  std::shared_ptr<MockQuadEncoder> forward =
      std::make_shared<MockQuadEncoder>(0);
  std::shared_ptr<MockQuadEncoder> lateral =
      std::make_shared<MockQuadEncoder>(0);
  std::shared_ptr<MockImu> imu = std::make_shared<MockImu>(0);
  std::shared_ptr<TwoRotationInertialOdometry> odom =
      std::make_shared<TwoRotationInertialOdometry>(
          forward, lateral, imu, wheel_diameter, wheel_diameter);
  AsyncRunner odomRunner(odom);
  OdometryState state = odom->get_state();

  cout << "Initial state" << endl;
  printOdomState(state, forward, lateral);

  for (int i = 0; i < 10; i++) {
    int val = rand() % 20000 + 1;

    state = odom->get_state();

    QLength expected_pos = state.pos.x + (val / 8192.0 * 360 * degree / radian *
                                          (wheel_diameter / 2));

    cout << "Incrementing " << val << " ticks" << endl;
    cout << "Expected val: " << expected_pos.convert(inch) << " inches" << endl;

    for (int i = 0; i < val; i++) {
      forward->increment();
      pros::delay(1);
      cout << "i = " << i << "\tForward = " << forward->get_value()
           << "\tLooparounds = " << forward->get_looparounds() << endl;
    }

    state = odom->get_state();

    QLength actual_pos = state.pos.x;
    cout << "Actual val: " << actual_pos.convert(inch) << " inches" << endl;

    ASSERT_NEAR(expected_pos.convert(inch), state.pos.x.convert(inch), 0.01);
  }
}

void rotationsensortest(QLength wheel_diameter) {
  std::shared_ptr<MockRotarySensor> forward =
      std::make_shared<MockRotarySensor>(0);
  std::shared_ptr<MockRotarySensor> lateral =
      std::make_shared<MockRotarySensor>(0);
  std::shared_ptr<MockImu> imu = std::make_shared<MockImu>(0);
  std::shared_ptr<TwoRotationInertialOdometry> odom =
      std::make_shared<TwoRotationInertialOdometry>(
          forward, lateral, imu, wheel_diameter, wheel_diameter);
  AsyncRunner odomRunner(odom);
  OdometryState state = odom->get_state();

  cout << "Initial state" << endl;
  printOdomState(state, forward, lateral);

  for (int i = 0; i < 10; i++) {
    int val = rand() % 20000 + 1;

    state = odom->get_state();

    QLength expected_pos = state.pos.x + (val / 36000.0 * 360 * degree /
                                          radian * (wheel_diameter / 2));

    cout << "Incrementing " << val << " ticks" << endl;
    cout << "Expected val: " << expected_pos.convert(inch) << " inches" << endl;

    for (int i = 0; i < val; i++) {
      forward->increment();
      pros::delay(1);
      cout << "i = " << i << "\tForward = " << forward->get_value() << endl;
    }

    state = odom->get_state();

    QLength actual_pos = state.pos.x;
    cout << "Actual val: " << actual_pos.convert(inch) << " inches" << endl;

    ASSERT_NEAR(expected_pos.convert(inch), state.pos.x.convert(inch), 0.01);
  }
}

// TEST(OdomTests, QuadEncoderTest) {
//   QLength wheel_diameter = 2.75_in;
//   quadencodertest(wheel_diameter);
// }

// TEST(OdomTests, RotarySensorTest) {
//   QLength wheel_diameter = 2.75_in;
//   rotationsensortest(wheel_diameter);
// }