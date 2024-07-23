#include "rev/api/alg/odometry/odometry.hh"

namespace rev {

/**
 * @brief Algebraic data type for fancy return matching
 *
 */
enum class SegmentStatusType { DRIVE, BRAKE, NEXT };
struct SegmentStatus {
  SegmentStatusType status;
  double power_left{0.};
  double power_right{0.};

  /**
   * @brief Return status for applying a pair of powers to the chassis
   *
   * @param ipower_left The power to apply to the left side of the chassis
   * @param ipower_right The power to apply to the right side of the chassis
   * @return SegmentStatus
   */
  static SegmentStatus drive(double ipower_left, double ipower_right) {
    SegmentStatus status;
    status.status = SegmentStatusType::DRIVE;
    status.power_left = ipower_left;
    status.power_right = ipower_right;

    return status;
  }

  /**
   * @brief Return status for applying power to the chassis
   *
   * @param ipower A power to apply to both sides of the chassis
   * @return SegmentStatus
   */
  static SegmentStatus drive(double ipower) { return drive(ipower, ipower); }

  /**
   * @brief Return status for applying a pair of powers to the chassis, as a
   * tuple
   *
   * @param ipowers A tuple of powers to apply
   * @return SegmentStatus
   */
  static SegmentStatus drive(std::tuple<double, double> ipowers) {
    double left, right;
    std::tie(left, right) = ipowers;
    return drive(left, right);
  }

  static SegmentStatus brake() {
    SegmentStatus status;
    status.status = SegmentStatusType::BRAKE;
    return status;
  }

  static SegmentStatus next() {
    SegmentStatus status;
    status.status = SegmentStatusType::NEXT;
    return status;
  }
};

/**
 * @brief Interface for path segments to be used with the Reckless controller
 *
 */
class RecklessSegment {
 public:
  /**
   * @brief Initialize the path segment
   *
   * This method is called once when execution of a path segment begins.
   *
   * The intended use of this method is to perform any calculations that need to
   * happen at the start of a segment.
   */
  virtual void init(OdometryState initial_state) = 0;

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
  virtual SegmentStatus step(OdometryState current_state) = 0;

  /**
   * @brief Clean-up
   *
   * Executes immediately after a NEXT value is returned.
   */
  virtual void clean_up() = 0;
};
}  // namespace rev
