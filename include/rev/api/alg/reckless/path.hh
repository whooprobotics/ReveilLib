#pragma once

#include <memory>
#include <vector>

#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/stop/stop.hh"  

namespace rev {
struct RecklessPathSegment {
    std::shared_ptr<Motion> motion;
    std::shared_ptr<Correction> correction;
    std::shared_ptr<Stop> stop;

    Position target_point;
    QLength drop_early;
};

struct RecklessPath {
    std::vector<RecklessPathSegment> segments;

    RecklessPath() {
        segments = std::vector<RecklessPathSegment>();
    }
};
}