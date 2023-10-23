#pragma once

#include <memory>

#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/stop/stop.hh"  

namespace rev {
struct RecklessPathSegment {
    std::unique_ptr<Motion> motion;
    std::unique_ptr<Correction> correction;
    std::unique_ptr<Stop> stop;
};

struct RecklessPath {

};
}