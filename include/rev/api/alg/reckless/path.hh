#pragma once

#include <memory>
#include <vector>

#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/stop/stop.hh"
#include "rev/api/alg/reckless/segment.hh"

namespace rev {

/**
 * @brief Complete path for use with the Reckless Controller
 *
 */
struct RecklessPath {
  std::vector<std::shared_ptr<RecklessSegment>> segments;

  RecklessPath() { segments = std::vector<std::shared_ptr<RecklessSegment>>(); }
  RecklessPath(std::initializer_list<std::shared_ptr<RecklessSegment>> seg) {
    for (auto& s : seg) {
      segments.push_back(s);
    }
  }
  /**
   * @brief Add a segment to the path under construction
   *
   * @param segment The segment to add
   * @return RecklessPath& An ongoing path builder
   */
  template <typename T>
  RecklessPath& with_segment(T segment) {
    static_assert(std::is_base_of<RecklessSegment, T>::value,
                  "with_segment parameter must implement RecklessSegment");
    segments.push_back(std::make_shared<T>(segment));
    return *this;
  }
};
}  // namespace rev