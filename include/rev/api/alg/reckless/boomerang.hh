#pragma once
#include <memory>
#include "rev/api/alg/reckless/path.hh"

namespace rev {
/**
 * @brief Parameters for a BoomerangSegment
 */
struct BoomerangSegmentParams {
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;
  double lead;
};

/**
 * @brief Boomerang controller
 *
 * This is essentially just a Pilons controller with an intermediate "carrot" point
 */
class BoomerangSegment : public RecklessSegment {
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;

  Position start_point;
  Position target_point;
  Position frozen_carrot_point;
  QLength drop_early;

  double part_progress{0.0};

  double lead;

  bool close {false};

  SegmentStatus last_status{SegmentStatus::drive(0, 0)};

 public:
  BoomerangSegment(std::shared_ptr<Motion> imotion,
                std::shared_ptr<Correction> icorrection,
                std::shared_ptr<Stop> istop,
                double ilead,
                Position itarget_point,
                QLength idrop_early = 0 * inch)
      : motion(imotion),
        correction(icorrection),
        stop(istop),
        target_point(itarget_point),
        drop_early(idrop_early),
        lead(ilead) {
    start_point = {0_in, 0_in, 0_deg};
  }

  BoomerangSegment(BoomerangSegmentParams iparams,
                Position itarget_point,
                QLength idrop_early = 0 * inch)
      : motion(iparams.motion),
        correction(iparams.correction),
        lead(iparams.lead),
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
   * If a return value of
   */
  SegmentStatus step(OdometryState current_state) override;

  /**
   * @brief Clean-up
   *
   * Executes immediately after a NEXT value is returned.
   */
  void clean_up() override;

  double progress() override;

  std::shared_ptr<RecklessSegment> operator&() {
    return std::make_shared<BoomerangSegment>(*this);
  }

  static std::shared_ptr<BoomerangSegment> create(BoomerangSegmentParams params,
                                               Position target_point,
                                               QLength drop_early = 0 * inch) {
    return std::make_shared<BoomerangSegment>(params, target_point, drop_early);
  }
};
}  // namespace rev