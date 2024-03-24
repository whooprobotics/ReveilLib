#include "rev/api/async/async_runner.hh"

namespace rev {
AsyncRunner::AsyncRunner(std::shared_ptr<AsyncRunnable> icontroller,
                         uint32_t itdelta)
     {
        icontroller->start_thread();
}

AsyncRunner::~AsyncRunner() {}
}