#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/odometry/odometry.hh"
#include "rev/api/v5/alg/slipstream/power.hh"

namespace rev {

enum class SlipstreamSegmentStatusType {DRIVE, BRAKE, NEXT, DUMMY };

struct SlipstreamSegmentStatus {
  SlipstreamSegmentStatusType status;
  SlipstreamPower power{0., 0., 0., 0., 0., 0., 0., 0.};

  static SlipstreamSegmentStatus drive(double ipower_front_left_forward,
      double ipower_front_left_steer,
      double ipower_front_right_forward,
      double ipower_front_right_steer,
      double ipower_rear_left_forward,
      double ipower_rear_left_steer,
      double ipower_rear_right_forward,
      double ipower_rear_right_steer) {
    SlipstreamSegmentStatus status;
    status.status = SlipstreamSegmentStatusType::DRIVE;
    
    status.power.front_left_forward = ipower_front_left_forward;
    status.power.front_right_forward = ipower_front_right_forward;
    status.power.rear_left_forward = ipower_rear_left_forward;
    status.power.rear_right_forward = ipower_rear_right_forward;

    status.power.front_left_steer = ipower_front_left_steer;
    status.power.front_right_steer = ipower_front_right_steer;
    status.power.rear_left_steer = ipower_rear_left_steer;
    status.power.rear_right_steer = ipower_rear_right_steer;

    return status;
  }

  static SlipstreamSegmentStatus drive(SlipstreamPower ipower) {
    SlipstreamSegmentStatus status;
    status.status = SlipstreamSegmentStatusType::DRIVE;
    status.power = ipower;
  return status;
  }
  
  static SlipstreamSegmentStatus brake() {
    SlipstreamSegmentStatus status;
    status.status = SlipstreamSegmentStatusType::BRAKE;
    return status;
  }

  static SlipstreamSegmentStatus next() {
    SlipstreamSegmentStatus status;
    status.status = SlipstreamSegmentStatusType::NEXT;
    return status;
  }

  static SlipstreamSegmentStatus dummy() {
    SlipstreamSegmentStatus status;
    status.status = SlipstreamSegmentStatusType::DUMMY;
    return status;
  }
};

class SlipstreamSegment {
 public:
  virtual void init(OdometryState initial_state) = 0;

  virtual SlipstreamSegmentStatus step(OdometryState current_state) = 0;

  virtual void clean_up() = 0;

  virtual double progress() = 0;
};
}  // namespace rev

#endif