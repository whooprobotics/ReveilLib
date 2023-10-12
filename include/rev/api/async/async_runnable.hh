#pragma once
namespace rev {
class AsyncRunnable {
 public:
  virtual void step() = 0;
};
}  // namespace rev