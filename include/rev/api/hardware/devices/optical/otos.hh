#pragma once
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/all_units.hh"
#include <string>
#include <vector>
#include <sstream>

namespace rev {

class OTOS {
  public:
    OTOS();
    double get_x();
    double get_y();
    double get_h();
    void reset(); // Resets position to (0, 0, 0)
    void update(); // Gets new values from sensor

  private:
    double x;
    double y;
    double h;

    std::string input;
    std::string token;
    std::vector<std::string> tokens;
    std::stringstream ss;

};
} // namespace rev
