
#include <functional>

namespace rev {

class callback {
  public:
    callback(std::function<void()> auton_callback,
              float percent);

    bool progress(float current_percent);

  private:
    std::function<void()> auton_callback;
    float percent;
};

} // namespace rev

