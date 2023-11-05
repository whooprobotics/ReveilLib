#pragma once

#include <memory>
#include <vector>

#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/stop/stop.hh"

namespace rev {
struct RecklessPathSegment {
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;

  Position start_point;
  Position target_point;
  QLength drop_early;

  RecklessPathSegment(std::shared_ptr<Motion> imotion,
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
};

struct RecklessPath {
  std::vector<RecklessPathSegment> segments;

  RecklessPath() { segments = std::vector<RecklessPathSegment>(); }
  RecklessPath& with_segment(RecklessPathSegment segment);
};
}  // namespace rev