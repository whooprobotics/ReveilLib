#pragma once
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/slipstream.hh"

namespace rev {

struct TurnParams {
  PIDParams turn_k = {.p = constants.turn_kp,
                      .i = constants.turn_ki,
                      .d = constants.turn_kd,
                      .starti = constants.turn_starti};

  settleParams turn_settle = {
      .settle_error = constants.turn_settle_error,
      .settle_time = constants.turn_settle_time,
      .large_settle_error = constants.turn_large_settle_error,
      .large_settle_time = constants.turn_large_settle_time};

  QAngle exit_error = constants.turn_exit_error;
  double min_speed = constants.turn_min_speed;
  double max_speed = constants.turn_max_speed;
  QTime timeout = constants.turn_timeout;
  QAngle offset = 0_deg;
};

class MecanumTurnToAngle : public SlipstreamSegment {
  QAngle target_angle;
  TurnParams p;

  PID turnPID;

  bool crossed = false;
  QAngle prev_error = 0_deg;
  QAngle prev_raw_error = 0_deg;

  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumTurnToAngle(QAngle itarget_angle,
                     TurnParams iparams = TurnParams{})
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
      TurnParams params = TurnParams{}) {
    return std::make_shared<MecanumTurnToAngle>(target_angle, params);
  }
};

}  // namespace rev
