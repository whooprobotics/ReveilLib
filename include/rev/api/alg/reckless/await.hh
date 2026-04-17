#pragma once
#include <functional>
#include <memory>
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/async/async_awaitable.hh"

namespace rev {
class Await : public RecklessSegment {
 public:
  /**
   * @brief Construct a new Await segment
   *
   * @param idependency The Awaitable which this will wait for
   */
  Await(std::shared_ptr<AsyncAwaitable> idependency);

  /**
   * @brief Initialize the path segment
   *
   * This method is called once when execution of a path segment begins. It is
   * unused in the case of the Awaitable segment.
   */
  void init(OdometryState initial_state) override;

  /**
   * @brief Calculate the next step
   *
   * For the Await segment, this always returns DUMMY, until the readiness state
   * reaches a status of true, at which point it returns NEXT
   */
  SegmentStatus step(OdometryState current_state) override;

  /**
   * @brief Clean-up
   *
   * Executes immediately after a NEXT value is returned. Unused by Await
   * segment.
   */
  void clean_up() override;

  double progress() override { return 0; }

 private:
  std::shared_ptr<AsyncAwaitable> dependency;
};
}  // namespace rev