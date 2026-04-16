#pragma once
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/slipstream.hh"

namespace rev {

class MecanumToDistance : public SlipstreamSegment {
  QLength distance;
  QAngle heading;
  Drive p;

  PID drivePID;
  PID headingPID;

  Position start_pos;

  double part_progress{0.0};
  SlipstreamSegmentStatus last_status{
      SlipstreamSegmentStatus::drive({0, 0, 0, 0})};

 public:
  MecanumToDistance(QLength idistance, Drive iparams = Drive{})
      : distance(idistance), p(iparams) {}

  void init(OdometryState initial_state) override;

  SlipstreamSegmentStatus step(OdometryState current_state) override;

  void clean_up() override;
  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumToDistance>(*this);
  }

  static std::shared_ptr<MecanumToDistance> create(
      QLength distance,
      Drive params = Drive{}) {
    return std::make_shared<MecanumToDistance>(distance, params);
  }
};

}  // namespace rev
