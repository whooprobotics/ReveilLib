#pragma once

#include <algorithm>

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

  void clamp_powers () {
    front_left_forward = std::clamp(front_left_forward, -1.0, 1.0);
    front_right_forward = std::clamp(front_right_forward, -1.0, 1.0);
    rear_left_forward = std::clamp(rear_left_forward, -1.0, 1.0);
    rear_right_forward = std::clamp(rear_right_forward, -1.0, 1.0);

    front_left_steer = std::clamp(front_left_steer, -1.0, 1.0);
    front_right_steer = std::clamp(front_right_steer, -1.0, 1.0);
    rear_left_steer = std::clamp(rear_left_steer, -1.0, 1.0);
    rear_right_steer = std::clamp(rear_right_steer, -1.0, 1.0);
  }

};

} // namespace rev
