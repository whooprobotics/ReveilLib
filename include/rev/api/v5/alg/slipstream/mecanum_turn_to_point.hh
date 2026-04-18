#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/slipstream/mecanum_turn_to_angle.hh"
#include "rev/api/v5/alg/slipstream/segment.hh"
#include "rev/api/v5/alg/slipstream/slipstream.hh"

namespace rev {

class MecanumTurnToPoint : public SlipstreamSegment {
  Position target_point;
  Turn p;

  MecanumTurnToAngle inner;

 public:
  MecanumTurnToPoint(Position itarget_point, Turn iparams = Turn{})
      : target_point(itarget_point), p(iparams), inner(0_deg, iparams) {}

  void init(OdometryState initial_state) override;

  SlipstreamSegmentStatus step(OdometryState current_state) override;

  void clean_up() override;
  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumTurnToPoint>(*this);
  }

  static std::shared_ptr<MecanumTurnToPoint> create(
      Position target_point, Turn params = Turn{}) {
    return std::make_shared<MecanumTurnToPoint>(target_point, params);
  }
};

}  // namespace rev

#endif