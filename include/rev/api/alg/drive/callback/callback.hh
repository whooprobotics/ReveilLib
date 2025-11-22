
#include <functional>

namespace rev {

/**
 * @brief Callback for auton tasks after a percent
 */
class Callback {
  public:
    /**
     * @brief Contruct a callback
     * @param auton_callback The void function to be called after 
     * percent threshold has been reached, will only be called once
     * @param percent The percent threshold where the callback will 
     * be executed 
     */
    Callback(std::function<void()> auton_callback,
              float percent);

    /**
     * @return True when the callback is called after current percent 
     * is past percent. False otherwise 
     */
    bool progress(float current_percent);

  std::shared_ptr<Callback> operator&() {
    return std::make_shared<Callback>(*this);
  }

  private:
    std::function<void()> auton_callback;
    float percent;
    bool called{false};
};

} // namespace rev

