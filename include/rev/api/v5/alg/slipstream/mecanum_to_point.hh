#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/slipstream/segment.hh"
#include "rev/api/v5/alg/slipstream/slipstream.hh"

namespace rev {

class MecanumToPoint : public SlipstreamSegment {
  Position start_point;
  Position target_point;
  Drive p;

  PID drivePID;
  PID headingPID;
  bool prev_line_settled = false;

  QAngle desired_heading = 0_deg;
  bool heading_locked = false;
  QAngle locked_heading = 0_deg;

  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumToPoint(Position itarget_point,
                 Drive iparams = Drive{})
      : target_point(itarget_point), p(iparams) {
    start_point = {0_in, 0_in, 0_deg};
  }

  void init(OdometryState initial_state) override;

  SlipstreamSegmentStatus step(OdometryState current_state) override;

  void clean_up() override;
  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumToPoint>(*this);
  }

  static std::shared_ptr<MecanumToPoint> create(
      Position target_point, Drive params = Drive{}) {
    return std::make_shared<MecanumToPoint>(target_point, params);
  }
};

}  // namespace rev

#endif