#pragma once

#include <memory>

#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/chassis.hh"
#include "rev/api/alg/reckless/path.hh"

namespace rev {

enum class RecklessStatus {
    ACTIVE,
    DONE
};

class Reckless : public AsyncRunnable {
    public:
    void step() override;


    private:
    std::shared_ptr<Chassis> chassis;
    RecklessPath current_path {NULL};
    RecklessStatus status {RecklessStatus::DONE};
};
}

