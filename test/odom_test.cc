#include <gtest/gtest.h>
#include <iostream>
#include "rev/api/hardware/devices/mock_rotary_sensor.hh"
#include "rev/api/hardware/devices/mock_quad_encoder.hh"
#include "rev/rev.hh"

using namespace rev;
using std::cout, std::endl;

void quadencodertest() {
  MockQuadEncoder forward(0);
  MockQuadEncoder lateral(0);

  for(int i = 0; i < 100; i++) {
    forward.increment();
  }

  ASSERT_NEAR(forward.get_value(), 100, 1);

  for(int i = 0; i < 900; i++) {
    lateral.increment();
  }

  ASSERT_NEAR(forward.get_value(), 900, 1);

  cout << "Forward value: \t" << forward.get_value() << endl;
  cout << "Forward angle: \t" << forward.get_position() << endl;
  cout << "Lateral value: \t" << lateral.get_value() << endl;
  cout << "Lateral angle: \t" << lateral.get_position() << endl;

  for(int i = 0; i < 200; i++) {
    forward.decrement();
  }

  ASSERT_NEAR(forward.get_value(), 8092, 1);

  for(int i = 0; i < 1100; i++) {
    lateral.decrement();
  }

  ASSERT_NEAR(forward.get_value(), 8092, 1);

  cout << "Forward value: \t" << forward.get_value() << endl;
  cout << "Forward angle: \t" << forward.get_position() << endl;
  cout << "Lateral value: \t" << lateral.get_value() << endl;
  cout << "Lateral angle: \t" << lateral.get_position() << endl;
}

TEST(OdomTests, QuadEncoderTest) {
  quadencodertest();
}