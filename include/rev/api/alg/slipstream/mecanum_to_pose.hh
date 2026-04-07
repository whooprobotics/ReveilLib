#pragma once
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/slipstream.hh"

namespace rev {

struct MecanumToPoseParams {
  PIDParams drive_k = {.p = constants.drive_kp,
                       .i = constants.drive_ki,
                       .d = constants.drive_kd,
                       .starti = constants.drive_starti};
  PIDParams turn_k = {.p = constants.turn_kp,
                      .i = constants.turn_ki,
                      .d = constants.turn_kd,
                      .starti = constants.turn_starti};

  settleParams drive_settle = {
      .settle_error = constants.drive_settle_error,
      .settle_time = constants.drive_settle_time,
      .large_settle_error = constants.drive_large_settle_error,
      .large_settle_time = constants.drive_large_settle_time};

  settleParams turn_settle = {
      .settle_error = constants.turn_settle_error,
      .settle_time = constants.turn_settle_time,
      .large_settle_error = constants.turn_large_settle_error,
      .large_settle_time = constants.turn_large_settle_time};

  QLength exit_error = constants.drive_exit_error;
  double min_speed = constants.drive_min_speed;
  double max_speed = constants.drive_max_speed;
  double turn_max_speed = constants.turn_max_speed;
  QTime timeout = constants.drive_timeout;
};

class MecanumToPose : public SlipstreamSegment {
  Position start_point;
  Position target_point;
  MecanumToPoseParams p;

  PID drivePID;
  PID turnPID;
  bool prev_line_settled = false;


  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumToPose(Position itarget_point, MecanumToPoseParams iparams = MecanumToPoseParams{})
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
                                               MecanumToPoseParams params) {
    return std::make_shared<MecanumToPose>(target_point, params);
  }
};

}  // namespace rev
