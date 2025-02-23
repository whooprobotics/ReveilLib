#pragma once

namespace rev {

class Gyroscope {
 public:
  virtual double get_heading() = 0;
  virtual bool is_calibrating() = 0;
};

}  // namespace rev