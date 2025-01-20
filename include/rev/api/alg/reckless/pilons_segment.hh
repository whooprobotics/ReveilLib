#pragma once
#include "rev/api/alg/reckless/path.hh"

namespace rev{
/**
 * @brief Path segment for use with Reckless controller
 *
 * TODO: Rename this to be more reflective of what it is specifically, but not
 * until a major release.
 */
class PilonsSegment : public RecklessSegment {
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;

  Position start_point;
  Position target_point;
  QLength drop_early;

  SegmentStatus last_status{SegmentStatus::drive(0, 0)};

 public:
  PilonsSegment(std::shared_ptr<Motion> imotion,
                      std::shared_ptr<Correction> icorrection,
                      std::shared_ptr<Stop> istop,
                      Position itarget_point,
                      QLength idrop_early = 0 * inch)
      : motion(imotion),
        correction(icorrection),
        stop(istop),
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
};
}