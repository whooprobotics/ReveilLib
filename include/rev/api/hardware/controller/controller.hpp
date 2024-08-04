#include <memory>

namespace rev {
enum class ControllerType { PRIMARY, PARTNER };

enum Button : unsigned short {
  A = 1 << 0,
  B = 1 << 1,
  X = 1 << 2,
  Y = 1 << 3,
  UP = 1 << 4,
  DOWN = 1 << 5,
  LEFT = 1 << 6,
  RIGHT = 1 << 7,
  L1 = 1 << 8,
  L2 = 1 << 9,
  R1 = 1 << 10,
  R2 = 1 << 11
};

class Controller {
 public:
  static std::shared_ptr<Controller> get_instance(
      ControllerType type = ControllerType::PRIMARY);

 private:
  Controller(ControllerType type);
  static std::shared_ptr<Controller> primary;
  static std::shared_ptr<Controller> partner;
}
}  // namespace rev