#pragma once
#include "rev/api/alg/PID/PID.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/slipstream.hh"

namespace rev {

struct MecanumToDistanceParams {
  PIDParams drive_k = {.p = constants.drive_kp,
                       .i = constants.drive_ki,
                       .d = constants.drive_kd,
                       .starti = constants.drive_starti};
  PIDParams heading_k = {.p = constants.turn_kp,
                         .i = constants.turn_ki,
                         .d = constants.turn_kd,
                         .starti = constants.turn_starti};

  settleParams drive_settle = {
      .settle_error = constants.drive_settle_error,
      .settle_time = constants.drive_settle_time,
      .large_settle_error = constants.drive_large_settle_error,
      .large_settle_time = constants.drive_large_settle_time};

  double min_speed = constants.drive_min_speed;
  double max_speed = constants.drive_max_speed;
  double turn_max_speed = constants.turn_max_speed;
  QTime timeout = constants.drive_timeout;
};

class MecanumToDistance : public SlipstreamSegment {
  QLength distance;
  QAngle heading;
  MecanumToDistanceParams p;

  PID drivePID;
  PID headingPID;

  Position start_pos;

  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumToDistance(QLength idistance, QAngle iheading,
                    MecanumToDistanceParams iparams = MecanumToDistanceParams{})
      : distance(idistance), heading(iheading), p(iparams) {}

  void init(OdometryState initial_state) override;

  SlipstreamSegmentStatus step(OdometryState current_state) override;

  void clean_up() override;
  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumToDistance>(*this);
  }

  static std::shared_ptr<MecanumToDistance> create(
      QLength distance, QAngle heading,
      MecanumToDistanceParams params = MecanumToDistanceParams{}) {
    return std::make_shared<MecanumToDistance>(distance, heading, params);
  }
};

}  // namespace rev
