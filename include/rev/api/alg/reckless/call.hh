#pragma once
#include <functional>
#include "rev/api/alg/reckless/segment.hh"

namespace rev {
class Call : public RecklessSegment {
 public:
  /**
   * @brief Construct a new Call segment
   *
   * @param icallback The function which will be invoked for this segment
   *
   * This function should be **non-blocking**. This means it should perform some
   * action and return ASAP, without any artificial delays or control loops. If
   * you want to use this to, say, start moving an arm controlled by a PID
   * toward a target, you should instead have a separate thread for that, and
   * use the Call segment to set a variable that tells that thread to begin
   * execution.
   */
  Call(std::function<void(void)> icallback);

  /**
   * @brief Initialize the path segment
   *
   * This method is called once when execution of a path segment begins. It is
   * unused in the case of the Call segment.
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
   * Executes immediately after a NEXT value is returned. Unused by Call
   * segment.
   */
  void clean_up() override;

  double progress() override { return 0; }

 private:
  std::function<void(void)> callback;
};
}  // namespace rev