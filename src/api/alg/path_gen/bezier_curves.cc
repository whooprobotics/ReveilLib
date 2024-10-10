#include "rev/api/alg/path_gen/bezier_curves.hh"
#include <tuple>
#include "rev/api/units/q_length.hh"
#include "rev/api/units/r_quantity.hh"
#include "rev/util/mathutil.hh"

namespace rev {

BezierSegment::BezierSegment(std::vector<PointVector> path_points, std::size_t resolution, QLength tolerance) {
  this->path_points = path_points;
  this->resolution = resolution;
  this->tolerance = tolerance;
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  this->current_idx = 0;

  for (std::size_t t = 0; t < this->resolution; ++t){
    t_value = (double) t / resolution;
    for (std::size_t layer = 1; layer < this->path_points.size(); ++layer){
      for (std::size_t idx = 0; layer + idx < this->path_points.size(); ++idx){
        this->path_points[idx].x = (1 - t_value) * this->path_points[idx].x + t_value * this->path_points[idx + 1].x;
        this->path_points[idx].y = (1 - t_value) * this->path_points[idx].y + t_value * this->path_points[idx + 1].y;
      }
    }
  }

}

SegmentStatus BezierSegment::step(OdometryState current_state){

  current_point = this->path_points[current_idx];

  if (current_idx >= this->path_points.size()) return SegmentStatus::brake();

  QLength distance = sqrt((current_state.pos.x - current_point.x)*(current_state.pos.x - current_point.x) + 
                               (current_state.pos.y - current_point.y)*(current_state.pos.y - current_point.y));

  if (distance < tolerance) ++current_idx;

  

  double outer_power = 0.0;
  QLength radius = calculate_radius(current_state.pos, current_point);
  double inner_power = outer_power * calculate_inside_ratio(24_in, radius).get_value();
  std::tuple<double, double> pows = std::make_tuple(0.0, 0.0);

  return SegmentStatus::drive(pows);
}

void BezierSegment::clean_up() {}

}