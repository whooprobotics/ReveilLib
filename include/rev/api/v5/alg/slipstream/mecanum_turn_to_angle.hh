#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/slipstream/segment.hh"
#include "rev/api/v5/alg/slipstream/slipstream.hh"

namespace rev {

class MecanumTurnToAngle : public SlipstreamSegment {
  QAngle target_angle;
  Turn p;

  PID turnPID;

  bool crossed = false;
  QAngle prev_error = 0_deg;
  QAngle prev_raw_error = 0_deg;

  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumTurnToAngle(QAngle itarget_angle,
                     Turn iparams = Turn{})
      : target_angle(itarget_angle), p(iparams) {}

  void init(OdometryState initial_state) override;

  SlipstreamSegmentStatus step(OdometryState current_state) override;

  void clean_up() override;
  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumTurnToAngle>(*this);
  }

  static std::shared_ptr<MecanumTurnToAngle> create(
      QAngle target_angle,
      Turn params = Turn{}) {
    return std::make_shared<MecanumTurnToAngle>(target_angle, params);
  }
};

}  // namespace rev

#endif