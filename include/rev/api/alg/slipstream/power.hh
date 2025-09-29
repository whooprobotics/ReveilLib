#pragma once

namespace rev {

struct SlipstreamPower {
  double front_left_forward;
  double front_right_forward;
  double rear_left_forward;
  double rear_right_forward;

  double front_left_steer = 0.0;
  double front_right_steer = 0.0;
  double rear_left_steer = 0.0;
  double rear_right_steer = 0.0;
};

} // namespace rev
