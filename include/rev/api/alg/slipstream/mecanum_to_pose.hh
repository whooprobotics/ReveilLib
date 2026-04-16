#pragma once
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/slipstream.hh"

namespace rev {

class MecanumToPose : public SlipstreamSegment {
  Position start_point;
  Position target_point;
  Drive p;

  PID drivePID;
  PID turnPID;
  bool prev_line_settled = false;


  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumToPose(Position itarget_point, Drive iparams = Drive{})
      : target_point(itarget_point), p(iparams) {
    start_point = {0_in, 0_in, 0_deg};
  }

  void init(OdometryState initial_state) override;

  SlipstreamSegmentStatus step(OdometryState current_state) override;

  void clean_up() override;
  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumToPose>(*this);
  }

  static std::shared_ptr<MecanumToPose> create(Position target_point,
                                               Drive params) {
    return std::make_shared<MecanumToPose>(target_point, params);
  }
};

}  // namespace rev
