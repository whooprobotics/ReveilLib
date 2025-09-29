#pragma once
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/stop/stop.hh"
#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/slipstream/slipstream.hh"
#include "rev/api/alg/slipstream/path.hh"

namespace rev {
/**
 * @brief params for a mecanum segment (same as MecanumSegment params)
 */

struct MecanumSegmentParams {
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;
};

/**
 * @bried Path segment for Slipstream controller
 *
 */
class MecanumSegment : public SlipstreamSegment {
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;

  Position start_point;
  Position target_point;
  QLength drop_early;

  double part_progress{0.0};

 public:
  MecanumSegment(std::shared_ptr<Motion> imotion,
                 std::shared_ptr<Correction> icorrection,
                 std::shared_ptr<Stop> istop,
                 Position itarget_point,
                 QLength idrop_early = 0_in)
      : motion(imotion),
        correction(icorrection),
        stop(istop),
        target_point(itarget_point),
        drop_early(idrop_early) {
    start_point = {0_in, 0_in, 0_deg};
  }

  MecanumSegment(MecanumSegmentParams iparams,
                 Position itarget_point,
                 QLength idrop_early = 0_in)
      : motion(iparams.motion),
        correction(iparams.correction),
        stop(iparams.stop),
        target_point(itarget_point),
        drop_early(idrop_early) {
    start_point = {0_in, 0_in, 0_deg};
  }

  /**
   * @brief Initialize the path segment
   *
   * This method is called once when execution of a path segment begins.
   *
   * The intended use of this method is to perform any calculations that need to
   * happen at the start of a segment.
   */
  void init(OdometryState initial_state) override;

  /**
   * @brief Calculate the next step
   *
   * This method is executed once per step of the controller, until the segment
   * is completed, beginning with the same cycle as init(), immediately after.
   *
   * Upon reaching a return value of NEXT, the controller is intended to step to
   * the next segment, or if no next segment is present, terminate execution.
   *
   */
  SlipstreamSegmentStatus step(OdometryState current_state) override;

  /**
   * @brief Clean-up
   *
   * Executes immediately after a NEXT value is returned.
   */
  void clean_up() override;

  double progress() override;

  std::shared_ptr<SlipstreamSegment> operator&() {
    return std::make_shared<MecanumSegment>(*this);
  }

  static std::shared_ptr<MecanumSegment> create(MecanumSegmentParams params,
                                                Position target_point,
                                                QLength drop_early = 0_in) {
    return std::make_shared<MecanumSegment>(params, target_point, drop_early);
  }
};
}  // namespace rev
