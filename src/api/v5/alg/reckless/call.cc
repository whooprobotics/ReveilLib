#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/reckless/call.hh"

namespace rev {

Call::Call(std::function<void(void)> icallback) : callback(icallback) {}

void Call::init(OdometryState initial_state) {}

SegmentStatus Call::step(OdometryState current_state) {
  this->callback();
  return SegmentStatus::next();
}

void Call::clean_up() {};

}  // namespace rev

#endif