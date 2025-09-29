#pragma once

#include <memory>
#include <vector>

#include "rev/api/alg/slipstream/segment.hh"

namespace rev {

struct SlipstreamPath {
  std::vector<std::shared_ptr<SlipstreamSegment>> segments;

  SlipstreamPath() {
    segments = std::vector<std::shared_ptr<SlipstreamSegment>>();
  }

  explicit SlipstreamPath(
   std::initializer_list<std::shared_ptr<SlipstreamSegment>> seg) {
    segments = std::vector<std::shared_ptr<SlipstreamSegment>>();

    for (auto& s: seg) {
      segments.push_back(s);
    }
  }

  template<typename T>
  SlipstreamPath& with_segment(T segment) {
    static_assert(std::is_base_of<SlipstreamSegment, T>::value,
                  "with_segment parameter must implement SlipstreamSegment");
    segments.push_back(std::make_shared<T>(segment));
    return *this;
  }
};

} // namespace rev
