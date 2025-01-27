#include "rev/api/hardware/devices/optical/otos.hh"
#include <iostream>
using std::cin, std::string, std::getline;

namespace rev {

// initialize sensor readings to 0
OTOS::OTOS() {
  x = 0;
  y = 0;
  h = 0;
}

double OTOS::get_x() {
  return x;
}

double OTOS::get_y() {
  return y;
}

double OTOS::get_h() {
  return h;
}

// reads in new sensor values from serial input
void OTOS::update() {
  getline(cin, this->input);
  ss.str(input);

  while(getline(ss, token, ' ')) {
    this->tokens.push_back(token);
  }

  if (tokens.empty()) { return; }
  
  ss.clear();

  x = stod(tokens.at(0));
  y = stod(tokens.at(1));
  h = stod(tokens.at(2));

  tokens.clear();
}

} // namespace rev