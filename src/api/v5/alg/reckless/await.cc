#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/reckless/await.hh"

namespace rev {

Await::Await(std::shared_ptr<AsyncAwaitable> idependency)
    : dependency(idependency) {}

void Await::init(OdometryState initial_state) {}

SegmentStatus Await::step(OdometryState current_state) {
  if (dependency->is_ready())
    return SegmentStatus::next();
  else
    return SegmentStatus::dummy();
}

void Await::clean_up(){};
}  // namespace rev

#endif