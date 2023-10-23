#pragma once

#include <memory>

#include "rev/api/async/async_runnable.hh"
#include "rev/api/hardware/chassis/chassis.hh"

namespace rev {
class Reckless : public AsyncRunnable {
    public:
    void step() override;


    private:
    std::shared_ptr<Chassis> chassis;
};
}